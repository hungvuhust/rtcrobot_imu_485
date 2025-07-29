#ifndef __vrobot_imu_HPP__
#define __vrobot_imu_HPP__

#include "serial/serial.h"
#include "vrobot_imu/params.hpp"
#include <atomic>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/timer.hpp>
#include <sensor_msgs/msg/detail/imu__struct.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>
#include <string>

#include "wit_c_sdk/REG.h"
#include "wit_c_sdk/wit_c_sdk.h"

// tf2
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp" // for doTransform
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

using sensor_msgs::msg::Imu;

class VrobotIMU : public rclcpp::Node {
private:
  rclcpp::Publisher<Imu>::SharedPtr imu_pub_;

  bool                                        is_init_{false};
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
  std::unique_ptr<tf2_ros::Buffer>            tf_buffer_;

  tf2::Matrix3x3  rot_matrix_;
  tf2::Quaternion quaternion_;

  // Serial parameters
  std::string serial_port_;
  int         serial_baud_;
  int         serial_timeout_;
  std::string topic_name_;

  // Serial
  static std::unique_ptr<serial::Serial> serial_;
  static std::atomic<uint8_t>            flag_;

  // IMU
  double acc_[3];
  double gyr_[3];
  double angle_[3];
  double mag_[3];

  double           acc_bias_[3] = {5.42756348e-02, 3.49316406e-03, 0.0};
  std::atomic_bool is_bias_set_{false};

  // Thread
  std::unique_ptr<std::thread> thread_;

public:
  explicit VrobotIMU();
  ~VrobotIMU();

  void init();

  void thread_poll();

  static void sensor_data_updata(uint32_t uiReg, uint32_t uiRegNum) {
    for (size_t i = 0; i < uiRegNum; i++) {
      switch (uiReg) {
      case AX:
      case AY:
      case AZ: flag_ |= (1 << ACC_UPDATE); break;
      case GX:
      case GY:
      case GZ: flag_ |= (1 << GYR_UPDATE); break;
      case HX:
      case HY:
      case HZ: flag_ |= (1 << ANGLE_UPDATE); break;
      case Roll:
      case Pitch:
      case Yaw: flag_ |= (1 << ANGLE_UPDATE); break;
      default: flag_ |= (1 << PRESS_UPDATE); break;
      }
      uiReg++;
    }
  }

  static void sensor_uart_send(uint8_t *p_data, uint32_t uiSize) {
    serial_->write(p_data, uiSize);
  }
};

#endif // __vrobot_imu_HPP__