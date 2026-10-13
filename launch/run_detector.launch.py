#!/usr/bin/env python3

import os
import yaml

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
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

    # image_transport of the bag/driver: defaults match the robot's live RealSense drivers;
    # pass raw for bags that recorded uncompressed images (e.g. bags/bags_benchmark)
    depth_transport_arg = DeclareLaunchArgument(
        'depth_transport', default_value='compressedDepth',
        description='image_transport for the depth image (compressedDepth, zstd, raw, ...)')
    depth_transport = LaunchConfiguration('depth_transport')

    color_transport_arg = DeclareLaunchArgument(
        'color_transport', default_value='compressed',
        description='image_transport for the color image (compressed or raw)')
    color_transport = LaunchConfiguration('color_transport')

    # camera_info the calibration node reads its live intrinsics from. The default is the node's
    # own (color camera_info, right for an aligned depth stream); a bag that only has the raw
    # depth image needs the DEPTH camera_info here, or ICP deprojects it with the wrong K.
    calibration_camera_info_topic_arg = DeclareLaunchArgument(
        'calibration_camera_info_topic', default_value='/front_camera/camera/color/camera_info',
        description='camera_info for calibration_icp_node live intrinsics')

    # Same for wall_detector_node, which estimates the ground height from the depth image: its K must be the one
    # of the depth image it reads (color on the robot, where depth is aligned; DEPTH on bags with raw depth only,
    # or the floor comes out ~0.7 m too high and everything below it is cut away). Defaults to the topic above.
    wall_camera_info_topic_arg = DeclareLaunchArgument(
        'wall_camera_info_topic', default_value=LaunchConfiguration('calibration_camera_info_topic'),
        description='camera_info for wall_detector_node ground estimation intrinsics')

    rviz_arg = DeclareLaunchArgument(
        'rviz', default_value='true',
        description='Launch RViz')

    env_arg = DeclareLaunchArgument(
        'env', default_value='outdoor',
        description='Parameter set: cfg/detector_param_jo_zotac_<env>.yaml (indoor or outdoor)')

    pkg_dir = get_package_share_directory('jo_detector')
    config_file = PathJoinSubstitution(
        [pkg_dir, 'cfg', ['detector_param_jo_zotac_', LaunchConfiguration('env'), '.yaml']])
    # launch-time values (topics, frames) are identical in both parameter sets, so they are
    # read from the outdoor file
    launch_config_file = os.path.join(pkg_dir, 'cfg', 'detector_param_jo_zotac_outdoor.yaml')
    rviz_config_file = os.path.join(pkg_dir, 'rviz', 'detector_jo_zotac.rviz')

    calibration_params = _load_node_params(launch_config_file, 'calibration_icp_node')
    detector_params = _load_node_params(launch_config_file, 'detector_node')
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
            {'depth_transport': depth_transport},
            {'jo_detector.camera_info_topic': LaunchConfiguration('calibration_camera_info_topic')},
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
            {'jo_detector.depth_transport': depth_transport},
            {'jo_detector.color_transport': color_transport},
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
            {'image_transport': color_transport},
        ],
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz_detector',
        output='screen',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(LaunchConfiguration('rviz')),
    )

    wall_detector_node = Node(
        package='jo_detector',
        executable='wall_detector_node',
        name='wall_detector_node',
        output='screen',
        parameters=[
            ParameterFile(config_file, allow_substs=True),
            {'use_sim_time': use_sim_time},
            {'wall_detector.depth_transport': depth_transport},
            {'wall_detector.camera_info_topic': LaunchConfiguration('wall_camera_info_topic')},
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
        env_arg,
        use_sim_time_arg,
        depth_transport_arg,
        color_transport_arg,
        calibration_camera_info_topic_arg,
        wall_camera_info_topic_arg,
        rviz_arg,
        pythonpath_action,
        static_tf_velodyne_to_camera_link_initial_guess,
        calibration_node,
        detector_node,
        wall_detector_node,
        yolo_node,
        rviz_node,
        odometry_tf_publisher_node,
    ])
