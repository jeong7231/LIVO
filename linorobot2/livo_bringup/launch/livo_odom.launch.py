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
        Node(
            package='livo_bringup',
            executable='imu_topic_remap',
            output='screen'
        ),
        Node(
            package='joy',
            executable='joy_node',
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
                'pointcloud.ordered_pc': 'true',
                'initial_reset': 'true',
                'enable_accel' : 'true',
                'enable_gyro' : 'true',
                'unite_imu_method' : '1',
                'enable_infra1' : 'true',
                'enable_infra2' : 'true',
            }.items()
        ),

        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource(PathJoinSubstitution(
        #         [FindPackageShare('linorobot2_navigation'), 'launch', 'rtab_color_livo.launch.py']
        #     ))
        # ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution(
                [FindPackageShare('linorobot2_description'), 'launch', 'description.launch.py']
            ))
        ),
    ])
