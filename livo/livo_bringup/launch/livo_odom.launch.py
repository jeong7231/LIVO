from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
       
        Node(
            package='livo_bringup',
            executable='odom_node',
            output='screen'
        ),
        Node(
            package='livo_bringup',
            executable='driver_node',
            output='screen'
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
