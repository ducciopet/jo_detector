#!/usr/bin/env python3

import os
import yaml

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterFile
from ament_index_python.packages import get_package_share_directory


def _load_node_params(config_path, node_name):
    try:
        with open(config_path, 'r', encoding='utf-8') as cfg:
            content = yaml.safe_load(cfg) or {}
        return content.get(node_name, {}).get('ros__parameters', {})
    except Exception:
        return {}


def generate_launch_description():

    odom_pub_arg = DeclareLaunchArgument(
        'odom_pub', default_value='false',
        description='Launch odometry TF publisher (odom->base_link) from /odometry/filtered')
    odom_pub = LaunchConfiguration('odom_pub')

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description='Use simulation clock')
    use_sim_time = LaunchConfiguration('use_sim_time')

    pkg_dir = get_package_share_directory('jo_detector')
    config_file = os.path.join(pkg_dir, 'cfg', 'detector_param_jo_zotac_outdoor.yaml')
    rviz_config_file = os.path.join(pkg_dir, 'rviz', 'detector_jo_zotac.rviz')

    calibration_params = _load_node_params(config_file, 'calibration_icp_node')
    detector_params = _load_node_params(config_file, 'detector_node')
    detector_onboard_params = detector_params.get('jo_detector', {}) if isinstance(detector_params, dict) else {}
    camera_frame_initial_guess = str(calibration_params.get('camera_frame_initial_guess', 'camera_initial_guess'))

    odom_topic = str(detector_onboard_params.get('odom_topic', '/glim_ros/odom_corrected'))
    lidar_frame = str(detector_onboard_params.get('tf_lidar_frame', 'velodyne'))
    scripts_dir = os.path.join(pkg_dir, 'scripts')
    yolo_dir = os.path.join(scripts_dir, 'yolo_detector')

    pythonpath_action = SetEnvironmentVariable(
        'PYTHONPATH',
        yolo_dir + os.pathsep + scripts_dir + os.pathsep + os.environ.get('PYTHONPATH', '')
    )

    static_tf_velodyne_to_camera_link_initial_guess = Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='calib_icp_tf_velodyne_to_camera_link_initial_guess',
            parameters=[{'use_sim_time': use_sim_time}],
            # arguments=[
            #     '--x', '0.218304037',
            #     '--y', '0.107423631',
            #     '--z', '-0.020496928',
            #     '--roll', '-1.576042033',
            #     '--pitch', '-0.037013778',
            #     '--yaw', '-1.578200049',
            #     '--frame-id', lidar_frame,
            #     '--child-frame-id', camera_frame_initial_guess
            # ]
            # arguments=[
            #     '--x', '0.167250',
            #     '--y', '0.133722',
            #     '--z', '-0.408870',
            #     '--roll', '-1.476451',
            #     '--pitch', '0.014814',
            #     '--yaw', '-1.583296',
            #     '--frame-id', lidar_frame,
            #     '--child-frame-id', camera_frame_initial_guess
            # ]
            arguments=['--x', '0.169686', '--y', '0.137954', '--z', '-0.410085', '--roll', '-1.476806', '--pitch', '0.015591', '--yaw', '-1.584388', '--frame-id', lidar_frame, '--child-frame-id', camera_frame_initial_guess  ] 
    )

    calibration_node = Node(
        package='jo_detector',
        executable='calibration_icp_node',
        name='calibration_icp_node',
        output='screen',
        parameters=[
            ParameterFile(config_file, allow_substs=True),
            {'use_sim_time': use_sim_time},
        ],
    )

    detector_node = Node(
        package='jo_detector',
        executable='detector_node',
        name='detector_node',
        output='screen',
        parameters=[
            ParameterFile(config_file, allow_substs=True),
            {'use_sim_time': use_sim_time},
            {'jo_detector.odom_topic': odom_topic},
        ],
    )

    yolo_node = Node(
        package='jo_detector',
        executable='yolov11_detector_node.py',
        name='yolov11_detector_node',
        output='screen',
        parameters=[
            ParameterFile(config_file, allow_substs=True),
            {'use_sim_time': use_sim_time},
        ],
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz_detector',
        output='screen',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': True}],
    )

    wall_detector_node = Node(
        package='jo_detector',
        executable='wall_detector_node',
        name='wall_detector_node',
        output='screen',
        parameters=[
            ParameterFile(config_file, allow_substs=True),
            {'use_sim_time': use_sim_time},
        ],
    )

    odometry_tf_publisher_node = Node(
        package='jo_detector',
        executable='odometry_tf_publisher_node',
        name='odometry_tf_publisher',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'odom_topic': '/odometry/filtered'},
        ],
        condition=IfCondition(odom_pub),
    )

    return LaunchDescription([
        odom_pub_arg,
        use_sim_time_arg,
        pythonpath_action,
        static_tf_velodyne_to_camera_link_initial_guess,
        calibration_node,
        detector_node,
        wall_detector_node,
        yolo_node,
        rviz_node,
        odometry_tf_publisher_node,
    ])
