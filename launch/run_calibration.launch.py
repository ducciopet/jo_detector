#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
from launch_ros.descriptions import ParameterFile

def generate_launch_description():
    pkg_dir = get_package_share_directory('jo_detector')
    
    # Path to parameters file
    config_file = os.path.join(pkg_dir, 'cfg', 'detector_param_jo_zotac.yaml')
    
    # Path to rviz config file (optional)
    rviz_config_file = os.path.join(pkg_dir, 'rviz', 'detector_debug.rviz')
    
    return LaunchDescription([
        # Calibration node
        Node(
            package='jo_detector',
            executable='calibration_node',
            name='calibration_node',
            output='screen',
            parameters=[
                ParameterFile(config_file, allow_substs=True),
                {'use_sim_time': True},
            ],
        ),
        
        # RViz visualization (optional - comment out if not needed)
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz_calibration',
            output='screen',
            arguments=['-d', rviz_config_file],
        ),

        # Static TF transforms (required for calibration node)
        
        # map -> base_link (identity)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',                        
            name='calib_tf_map_to_base_link',
            arguments=[
                '--x','0.0',
                '--y','0.0',
                '--z','0.0',
                '--roll','0.0',
                '--pitch','0.0',
                '--yaw','0.0',
                '--frame-id','map', 
                '--child-frame-id','base_link'
            ]
        ),
        
        # base_link -> imu_link (identity)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',                        
            name='calib_tf_base_link_to_imu_link',
            arguments=[
                '--x','0.0',
                '--y','0.0',
                '--z','0.0',
                '--roll','0.0',
                '--pitch','0.0',
                '--yaw','0.0',
                '--frame-id','base_link', 
                '--child-frame-id','imu_link'
            ]
        ),

        # imu_link -> velodyne
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',                        
            name='calib_tf_imu_link_to_velodyne',
            arguments=[
                '--x','-0.1',
                '--y','0.0',
                '--z','0.30',
                '--roll','0.0',
                '--pitch','0.0',
                '--yaw','0.0',
                '--frame-id','imu_link', 
                '--child-frame-id','velodyne'
            ]
        ),

        # velodyne -> rs1_link (calibrated transform)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='calib_tf_velodyne_to_rs1_link',
            parameters=[{'use_sim_time': True}],
            arguments=[
                '--x','0.2',
                '--y','0.0',
                '--z','-0.65',
                '--roll','-1.4661566652919380',
                '--pitch','0.0',
                '--yaw','-1.8500490070139893',
                '--frame-id','velodyne',
                '--child-frame-id','rs1_link'
            ]
        ),

        # rs1_link -> camera_depth_optical_frame (identity, same physical position)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='calib_tf_rs1_link_to_camera_depth_optical_frame',
            parameters=[{'use_sim_time': True}],
            arguments=[
                '--x','0.0',
                '--y','0.0',
                '--z','0.0',
                '--roll','0.0',
                '--pitch','0.0',
                '--yaw','0.0',
                '--frame-id','rs1_link',
                '--child-frame-id','camera_depth_optical_frame'
            ]
        ),

        # rs1_link -> camera_color_optical_frame (identity, assuming aligned)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='calib_tf_rs1_link_to_camera_color_optical_frame',
            parameters=[{'use_sim_time': True}],
            arguments=[
                '--x','0.0',
                '--y','0.0',
                '--z','0.0',
                '--roll','0.0',
                '--pitch','0.0',
                '--yaw','0.0',
                '--frame-id','rs1_link',
                '--child-frame-id','camera_color_optical_frame'
            ]
        ),
    ])
