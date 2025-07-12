import os
from launch import LaunchDescription, Substitution, LaunchContext
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch_ros.actions import SetParameter
from launch.conditions import IfCondition, UnlessCondition
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    rtabmap_launch_path = PathJoinSubstitution(
        [FindPackageShare('rtabmap_launch'), 'launch', 'rtabmap.launch.py']
    )
    parameters=[{
          'frame_id':'camera_link',
          'subscribe_depth':True,
          'subscribe_odom_info':True,
          'approx_sync':False,
          'wait_imu_to_init':True}]

    return LaunchDescription([
        DeclareLaunchArgument(
            name='rviz', 
            default_value='false',
            description='Run rviz'
        ),
        DeclareLaunchArgument(
            name='localization', 
            default_value='false',
            description='run localization'
        ),
        DeclareLaunchArgument(
            name='rtabmap_viz', 
            default_value='false',
            description='Run rtabmap_viz'
        ),
        DeclareLaunchArgument(
            name='stereo', 
            default_value='false',
            description='Run stereo'
        ),
        Node(
            package='rtabmap_util', executable='point_cloud_xyz', output='screen',
            parameters=[{'approx_sync':False}],
            remappings=[('depth/image',       '/camera/camera/depth/image_rect_raw'),
                        ('depth/camera_info', '/camera/camera/depth/camera_info'),
                        ('cloud',             '/camera/camera/cloud_from_depth')]),
        
        # Generate aligned depth to color camera from the point cloud above       
        Node(
            package='rtabmap_util', executable='pointcloud_to_depthimage', output='screen',
            parameters=[{ 'decimation':2,
                          'fixed_frame_id':'camera_link',
                          'fill_holes_size':1}],
            remappings=[('camera_info', '/camera/camera/color/camera_info'),
                        ('cloud',       '/camera/camera/cloud_from_depth'),
                        ('image_raw',   '/camera/camera/realigned_depth_to_color/image_raw')]),
        
        # Compute quaternion of the IMU
        Node(
            package='imu_filter_madgwick', executable='imu_filter_madgwick_node', output='screen',
            parameters=[{'use_mag': False, 
                         'world_frame':'enu', 
                         'publish_tf':False}],
            remappings=[('imu/data_raw', '/camera/camera/imu')]),
        
        # The IMU frame is missing in TF tree, add it:
        Node(
            package='tf2_ros', executable='static_transform_publisher', output='screen',
            arguments=['0', '0', '0', '0', '0', '0', 'camera_gyro_optical_frame', 'camera_imu_optical_frame']),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(rtabmap_launch_path),
            launch_arguments={
                'localization': LaunchConfiguration("localization"),
                'rtabmap_viz' : LaunchConfiguration("rtabmap_viz"),
                'rviz' : LaunchConfiguration("rviz"),
                'depth' : 'true',
                'frame_id' : 'base_link',         
                'odom_frame_id' : '',                   
                'map_frame_id' : 'map',               
                'map_topic' : 'map',              
                'publish_tf_map' : 'true',              
                'namespace' : 'rtabmap',         
                'database_path' : '~/.ros/rtabmap.db', 
                'topic_queue_size' : '1',              
                'queue_size' : '10',                
                'qos' : '2',   
                'qos_scan' : '2',              
                'wait_for_transform' : '0.1',   
                'use_action_for_goal' : 'true',        
                'rgb_topic' : '/camera/camera/color/image_raw', 
                'depth_topic' : '/camera/camera/realigned_depth_to_color/image_raw',
                'camera_info_topic' : '/camera/camera/color/camera_info',  
                'imu_topic' : '/camera/camera/imu', 
                'wait_imu_to_init' : 'true', 
                'odom_topic' : '/odom/unfiltered',
                'subscribe_scan' : 'false',
                'scan_topic' : '/scan',
                'stereo' : LaunchConfiguration("stereo"),
                'left_image_topic' : '/camera/camera/infra1/image_rect_raw',
                'right_image_topic' : '/camera/camera/infra2/image_rect_raw',
                'left_camera_info_topic' : '/camera/camera/infra1/camera_info',
                'right_camera_info_topic' : '/camera/camera/infra2/camera_info',
            }.items()
        ),
    ])
