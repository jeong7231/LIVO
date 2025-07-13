from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    ekf_config_path = PathJoinSubstitution(
        [FindPackageShare('livo_base'), 'config', 'ekf_fixed.yaml']
    )

    return LaunchDescription([
       
        Node(
            package='livo_bringup',
            executable='odom_node_fixed', # MODIFIED: Use the fixed odom node
            name='odom_wheel_node',
            output='screen'
        ),
        Node(
            package='livo_bringup',
            executable='driver_node',
            output='screen'
        ),

        # EKF Filter Node
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[ekf_config_path]
        ),

        # YDLidar 
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution(
                [FindPackageShare('ydlidar_ros2_driver'), 'launch', 'ydlidar_launch.py']
            ))
        ),

        # Realsense 
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution(
                [FindPackageShare('realsense2_camera'), 'launch', 'rs_launch.py']
            )),
            launch_arguments={
                'pointcloud.enable': 'true',
                'ordered_pc': 'true',
                'initial_reset': 'true'
            }.items()
        ),
    ])
