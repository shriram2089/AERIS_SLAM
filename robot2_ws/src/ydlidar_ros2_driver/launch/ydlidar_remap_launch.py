#!/usr/bin/python3
# Copyright 2020, EAIBOT
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import LifecycleNode
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.actions import LogInfo

import lifecycle_msgs.msg
import os

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ydlidar_ros2_driver',
            executable='ydlidar_ros2_driver_node',
            name='ydlidar_node',
            output='screen',
            remappings=[
                ('/scan', '/robot1/scan')
            ],
            parameters=[{
                'port': '/dev/ttyUSB0',  # Change if your LiDAR uses a different port
                'frame_id': 'laser_frame',
                'ignore_array': '',
                'baudrate': 230400,
                'lidar_type': 'triangulation',
                'device_type': 'serial',
                'sample_rate': 9,
                'abnormal_check_count': 4,
                'resolution_fixed': True,
                'auto_reconnect': True,
                'isSingleChannel': False,
                'intensity': False,
                'support_motor_dtr': True,
                'angle_max': 180.0,
                'angle_min': -180.0,
                'range_max': 12.0,
                'range_min': 0.1,
                'frequency': 10.0,
                'invalid_range_is_inf': False
            }]
        )
    ])
