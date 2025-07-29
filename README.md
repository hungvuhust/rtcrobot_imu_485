# VRobot IMU Driver

Driver ROS2 cho cảm biến IMU sử dụng giao tiếp RS485/Serial, hỗ trợ xuất dữ liệu cảm biến quán tính (gia tốc, con quay hồi chuyển, từ trường) qua ROS2 topic.

## Mô tả

Package này cung cấp driver ROS2 cho cảm biến IMU giao tiếp qua cổng serial. Driver đọc dữ liệu từ cảm biến IMU và xuất bản dữ liệu dưới dạng `sensor_msgs::msg::Imu` message.

### Tính năng chính

- Đọc dữ liệu gia tốc (acceleration)
- Đọc dữ liệu con quay hồi chuyển (angular velocity)  
- Đọc dữ liệu góc nghiêng (orientation)
- Đọc dữ liệu từ trường (magnetic field)
- Hỗ trợ transform frame từ `imu_link` sang `base_link`
- Bias correction cho dữ liệu gia tốc

## Yêu cầu hệ thống

- ROS2 (Humble/Iron/Rolling)
- Ubuntu 20.04/22.04
- CMake 3.8+
- Cảm biến IMU hỗ trợ giao thức WIT

## Dependencies

### ROS2 Packages
- `rclcpp`
- `sensor_msgs`
- `geometry_msgs`
- `std_msgs`
- `tf2`
- `tf2_ros`
- `tf2_geometry_msgs`

### Thư viện bên thứ 3
- **Serial Library**: Thư viện giao tiếp serial (`3rd/serial/`)
- **WIT C SDK**: SDK cho cảm biến WIT (`3rd/wit_c_sdk/`)

## Cài đặt

### 1. Clone repository
```bash
cd ~/ros2_ws/src
git clone <repository-url> rtcrobot_imu_485
```

### 2. Build package
```bash
cd ~/ros2_ws
colcon build --packages-select vrobot_imu
```

### 3. Source workspace
```bash
source ~/ros2_ws/install/setup.bash
```

## Cấu hình

### ROS2 Parameters

Node hỗ trợ các parameters sau:

| Parameter | Kiểu dữ liệu | Giá trị mặc định | Mô tả |
|-----------|--------------|------------------|-------|
| `serial_port` | string | `/dev/imu` | Đường dẫn đến cổng serial |
| `serial_baud` | int | `9600` | Tốc độ baud rate |
| `serial_timeout` | int | `1000` | Thời gian timeout cho serial (ms) |
| `topic_name` | string | `/imu` | Tên topic xuất bản dữ liệu IMU |

### Cách sử dụng parameters

#### 1. Truyền parameters qua command line
```bash
ros2 run vrobot_imu vrobot_imu_node --ros-args -p serial_port:=/dev/ttyUSB0 -p serial_baud:=115200
```

#### 2. Sử dụng file yaml
Tạo file `config/imu_params.yaml`:
```yaml
vrobot_imu:
  ros__parameters:
    serial_port: "/dev/ttyUSB0"
    serial_baud: 115200
    serial_timeout: 2000
    topic_name: "/robot/imu"
```

Chạy với file config:
```bash
ros2 run vrobot_imu vrobot_imu_node --ros-args --params-file config/imu_params.yaml
```

#### 3. Set parameters runtime
```bash
# Kiểm tra parameters hiện tại
ros2 param list /vrobot_imu

# Đặt parameter mới
ros2 param set /vrobot_imu serial_baud 115200

# Lấy giá trị parameter
ros2 param get /vrobot_imu serial_port
```

### Tham số cũ (deprecated - chỉ để tham khảo)

Các tham số cũ trong `include/vrobot_imu/params.hpp` vẫn được giữ làm giá trị mặc định:

| Tham số | Giá trị mặc định | Mô tả |
|---------|------------------|-------|
| `SERIAL_PORT` | `/dev/imu` | Đường dẫn đến cổng serial |
| `SERIAL_BAUD` | `9600` | Tốc độ baud rate |
| `SERIAL_TIMEOUT` | `1000` ms | Thời gian timeout cho serial |
| `TOPIC_NAME` | `/imu` | Tên topic xuất bản dữ liệu IMU |
| `PUBLISH_RATE` | `100` Hz | Tần suất xuất bản dữ liệu |

### Thiết lập quyền truy cập serial
```bash
sudo chmod 666 /dev/ttyUSB0  # hoặc port tương ứng
# hoặc thêm user vào group dialout
sudo usermod -a -G dialout $USER
```

## Chạy node

### Chạy trực tiếp
```bash
ros2 run vrobot_imu vrobot_imu_node
```

### Chạy với parameters từ command line
```bash
ros2 run vrobot_imu vrobot_imu_node --ros-args -p serial_port:=/dev/ttyUSB0 -p serial_baud:=115200
```

### Chạy với launch file
```bash
ros2 launch vrobot_imu imu.launch.py
```

Launch file sẽ tự động load config từ `config/imu_params.yaml` và chạy node với các parameters đã định sẵn.

## Topics

### Published Topics

| Topic | Message Type | Mô tả |
|-------|-------------|-------|
| `/imu` | `sensor_msgs/msg/Imu` | Dữ liệu IMU (gia tốc, velocity góc, orientation) |

### Message Format

```bash
# Xem dữ liệu realtime
ros2 topic echo /imu

# Kiểm tra thông tin topic
ros2 topic info /imu
```

## Transform Frames

Node sử dụng TF2 để transform dữ liệu từ `imu_link` frame sang `base_link` frame. Đảm bảo rằng transform này đã được định nghĩa trong hệ thống.

```bash
# Kiểm tra transform tree
ros2 run tf2_tools view_frames.py
```

## Cấu trúc dự án

```
rtcrobot_imu_485/
├── 3rd/                    # Thư viện bên thứ 3
│   ├── serial/             # Thư viện serial communication
│   └── wit_c_sdk/          # SDK cho cảm biến WIT
├── config/                 # Configuration files
│   └── imu_params.yaml     # ROS2 parameters config
├── include/vrobot_imu/     # Header files
│   ├── params.hpp          # Tham số cấu hình (deprecated)
│   └── vrobot_imu.hpp      # Class chính
├── launch/                 # Launch files
│   └── imu.launch.py       # Launch file chính
├── src/
│   └── vrobot_imu.cpp      # Implementation chính
├── CMakeLists.txt          # Build configuration
├── package.xml             # ROS2 package metadata
└── README.md               # Documentation
```

## Troubleshooting

### Lỗi thường gặp

1. **Serial port không mở được**
   ```
   Serial port is not open
   ```
   - Kiểm tra device path: `ls /dev/tty*`
   - Kiểm tra quyền truy cập: `sudo chmod 666 /dev/ttyUSB0`

2. **Transform error**
   ```
   Transform error: Could not find transform
   ```
   - Đảm bảo TF tree đã được setup đúng
   - Kiểm tra static transform publisher

3. **Build errors**
   - Kiểm tra dependencies đã được cài đặt đầy đủ
   - Clean build: `colcon build --packages-select vrobot_imu --cmake-clean-cache`

### Debug commands

```bash
# Kiểm tra serial ports
ls /dev/tty*

# Monitor serial data
sudo cat /dev/ttyUSB0

# Kiểm tra ROS2 nodes
ros2 node list

# Kiểm tra topics
ros2 topic list
```

## Thông tin liên hệ

- **Maintainer**: rtc <agv04@rtc.edu.vn>
- **Version**: 0.0.0
- **License**: TODO

## Đóng góp

1. Fork project
2. Tạo feature branch (`git checkout -b feature/AmazingFeature`)
3. Commit changes (`git commit -m 'Add some AmazingFeature'`)
4. Push to branch (`git push origin feature/AmazingFeature`)
5. Mở Pull Request 