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
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        # Realsense 센서 매개변수 선언
        DeclareLaunchArgument(
            name='pointcloud_enable',
            default_value='true',
            description='Enable point cloud generation'
        ),

        DeclareLaunchArgument(
            name='ordered_pc',
            default_value='true',
            description='Enable ordered point cloud'
        ),

        DeclareLaunchArgument(
            name='initial_reset',
            default_value='true',
            description='Perform an initial reset of the device'
        ),

        # Realsense 노드 실행
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution(
                [FindPackageShare('realsense2_camera'), 'launch', 'rs_launch.py']
            )),
            launch_arguments={
                'pointcloud.enable': LaunchConfiguration('pointcloud_enable'),
                'ordered_pc': LaunchConfiguration('ordered_pc'),
                'initial_reset': LaunchConfiguration('initial_reset')
            }.items()
        )
    ])
