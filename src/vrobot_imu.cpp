#include "vrobot_imu/vrobot_imu.hpp"
#include "vrobot_imu/params.hpp"
#include <cstdio>
#include <functional>
#include <memory>
#include <rclcpp/logging.hpp>
#include <sensor_msgs/msg/detail/imu__struct.hpp>
#include <tf2/LinearMath/Vector3.h>

std::unique_ptr<serial::Serial> VrobotIMU::serial_ = nullptr;
std::atomic<uint8_t>            VrobotIMU::flag_   = 0;

VrobotIMU::VrobotIMU() : Node("vrobot_imu") {
  // Declare parameters with default values
  this->declare_parameter("serial_port", std::string("/dev/imu"));
  this->declare_parameter("serial_baud", 9600);
  this->declare_parameter("serial_timeout", 1000);
  this->declare_parameter("topic_name", std::string("/imu"));
  this->declare_parameter("base_link_frame", std::string("base_link"));
  this->declare_parameter("imu_link_frame", std::string("imu_link"));

  // Get parameters
  serial_port_     = this->get_parameter("serial_port").as_string();
  serial_baud_     = this->get_parameter("serial_baud").as_int();
  serial_timeout_  = this->get_parameter("serial_timeout").as_int();
  topic_name_      = this->get_parameter("topic_name").as_string();
  base_link_frame_ = this->get_parameter("base_link_frame").as_string();
  imu_link_frame_  = this->get_parameter("imu_link_frame").as_string();

  RCLCPP_INFO(this->get_logger(), "Serial port: %s", serial_port_.c_str());
  RCLCPP_INFO(this->get_logger(), "Serial baud: %d", serial_baud_);
  RCLCPP_INFO(this->get_logger(), "Serial timeout: %d ms", serial_timeout_);
  RCLCPP_INFO(this->get_logger(), "Topic name: %s", topic_name_.c_str());
  RCLCPP_INFO(this->get_logger(), "Base link frame: %s",
              base_link_frame_.c_str());
  RCLCPP_INFO(this->get_logger(), "IMU link frame: %s",
              imu_link_frame_.c_str());

  imu_pub_ = create_publisher<Imu>(topic_name_, 10);

  tf_buffer_   = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  if (!serial_) {
    serial_ = std::make_unique<serial::Serial>(
        serial_port_, serial_baud_,
        serial::Timeout::simpleTimeout(serial_timeout_));
  }

  if (!serial_->isOpen()) {
    RCLCPP_ERROR(this->get_logger(), "Serial port is not open");
    return;
  }

  init();

  thread_ =
      std::make_unique<std::thread>(std::bind(&VrobotIMU::thread_poll, this));
}

VrobotIMU::~VrobotIMU() {
  if (thread_) {
    if (thread_->joinable()) {
      thread_->join();
    }
  }
}

void VrobotIMU::init() {
  WitInit(WIT_PROTOCOL_NORMAL, 0x50);
  WitRegisterCallBack(VrobotIMU::sensor_data_updata);
  WitSerialWriteRegister(VrobotIMU::sensor_uart_send);

  WitWriteReg(0x01, 0x01);
  std::this_thread::sleep_for(std::chrono::milliseconds(1000));
  WitWriteReg(0x01, 0x00);
}

void VrobotIMU::thread_poll() {
  while (rclcpp::ok()) {

    if (!is_init_) {
      geometry_msgs::msg::TransformStamped transform;
      try {
        transform = tf_buffer_->lookupTransform(
            base_link_frame_, imu_link_frame_, tf2::TimePointZero);

        quaternion_.setValue(
            transform.transform.rotation.x, transform.transform.rotation.y,
            transform.transform.rotation.z, transform.transform.rotation.w);
        rot_matrix_.setRotation(quaternion_);

        is_init_ = true;
      } catch (const std::exception &ex) {
        RCLCPP_ERROR(this->get_logger(), "Transform error: %s", ex.what());
        continue;
      }
    }

    WitReadReg(AX, 12);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    while (serial_->available()) {
      auto cBuff = serial_->read(1);
      WitSerialDataIn(cBuff[0]);
    }

    if (flag_) {

      flag_ = 0;

      for (int i = 0; i < 3; i++) {
        acc_[i]   = sReg[AX + i] / 32768.0f * 16.0f * 9.8;  // m/s^2
        gyr_[i]   = sReg[GX + i] / 32768.0f * 34.90658504f; // rad/s
        angle_[i] = sReg[Roll + i] / 32768.0f * 3.14159265358979323846f; // rad
      }

      mag_[0] = sReg[HX];
      mag_[1] = sReg[HY];
      mag_[2] = sReg[HZ];

      tf2::Quaternion q_imu;
      q_imu.setRPY(angle_[0], angle_[1], angle_[2]); // Quaternion from IMU

      tf2::Quaternion q_base = quaternion_; // Quaternion from base_link
      q_base *= q_imu; // Quaternion from base_link * Quaternion from IMU
      q_base.normalize();

      // Transform angular velocity
      tf2::Vector3 angular_velocity_imu(gyr_[0], gyr_[1], gyr_[2]);
      tf2::Vector3 angular_velocity_base =
          rot_matrix_ * angular_velocity_imu; // Transform to base_link frame

      // Transform acceleration
      tf2::Vector3 acc_imu(acc_[0], acc_[1], acc_[2]);
      tf2::Vector3 acc_base =
          rot_matrix_ * acc_imu; // Transform to base_link frame

      if (!is_bias_set_) {
        acc_bias_[0] = acc_base.x();
        acc_bias_[1] = acc_base.y();
        is_bias_set_ = true;
        RCLCPP_INFO(this->get_logger(), "Bias set");
      }

      sensor_msgs::msg::Imu imu_msg;
      imu_msg.header.stamp    = this->now();
      imu_msg.header.frame_id = base_link_frame_;

      imu_msg.linear_acceleration.x = acc_base.x() - acc_bias_[0];
      imu_msg.linear_acceleration.y = acc_base.y() - acc_bias_[1];
      imu_msg.linear_acceleration.z = acc_base.z();

      imu_msg.angular_velocity.x = angular_velocity_base.x();
      imu_msg.angular_velocity.y = angular_velocity_base.y();
      imu_msg.angular_velocity.z = angular_velocity_base.z();

      // Quaternion
      imu_msg.orientation.x = q_base.x();
      imu_msg.orientation.y = q_base.y();
      imu_msg.orientation.z = q_base.z();
      imu_msg.orientation.w = q_base.w();

      imu_pub_->publish(imu_msg);
    }
  }
}

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<VrobotIMU>());
  rclcpp::shutdown();
  return 0;
}