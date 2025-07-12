# Copyright (c) 2021 Juan Miguel Jimeno
#
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

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # YDLiDAR 센서 매개변수 선언
        DeclareLaunchArgument(
            name='topic_name', 
            default_value='scan',
            description='Laser Topic Name'
        ),

        DeclareLaunchArgument(
            name='frame_id', 
            default_value='laser',
            description='Laser Frame ID'
        ),

        DeclareLaunchArgument(
            name='port', 
            default_value='/dev/ydlidar',
            description='YDLiDAR serial port device name'
        ),

        DeclareLaunchArgument(
            name='baudrate', 
            default_value='128000',
            description='YDLiDAR serial port baudrate'
        ),

        DeclareLaunchArgument(
            name='frequency',
            default_value='5.0',
            description='YDLiDAR scan frequency'
        ),

        # YDLiDAR 노드 실행
        Node(
            package='ydlidar_ros2_driver',
            executable='ydlidar_ros2_driver_node',
            name='ydlidar_ros2_driver_node',
            output='screen',
            emulate_tty=True,
            remappings=[('scan', LaunchConfiguration('topic_name'))],
            parameters=[{
                'port': LaunchConfiguration('port'),
                'frame_id': LaunchConfiguration('frame_id'),
                'baudrate': LaunchConfiguration('baudrate'),
                'frequency': LaunchConfiguration('frequency'),
                'ignore_array': '',
                'lidar_type': 1,
                'device_type': 6,
                'sample_rate': 5,
                'abnormal_check_count': 4,
                'fixed_resolution': True,
                'reversion': False,
                'inverted': True,
                'auto_reconnect': True,
                'isSingleChannel': False,
                'intensity': False,
                'support_motor_dtr': True,
                'angle_max': 60.0,
                'angle_min': -60.0,
                'range_max': 10.0,
                'range_min': 0.12,
                'invalid_range_is_inf': False
            }]
        ),
    ])
