#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Lấy đường dẫn đến package
    package_dir = get_package_share_directory('vrobot_imu')

    # Đường dẫn đến file config
    config_file = os.path.join(package_dir, 'config', 'imu_params.yaml')

    imu = Node(
        package='vrobot_imu',
        executable='vrobot_imu_node',
        name='vrobot_imu',
        output='screen',
        parameters=[config_file],
    )

    return LaunchDescription([
        imu,
    ])
