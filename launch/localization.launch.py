import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Get the launch directory
    pkg_dir = get_package_share_directory('jo_navigation')


    declare_params_file_cmd = DeclareLaunchArgument(
        'localization_params',
        default_value=os.path.join(pkg_dir, 'config', 'localization.yaml'),
        description='Full path to the ROS2 parameters file to use for all launched nodes')
    
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true')

    # Passed straight through to visodom.launch.py — see its own
    # declarations for the alternative sim-raw defaults it falls back to
    # when nothing is passed in standalone. This file's only caller
    # (localization_detector.launch.py) is for bag/real replay, not pure
    # sim (sim already gets glim for free from launch_sim.launch.py's own
    # delayed_glim, without this file at all) — so these default straight
    # to the real camera's own topic names/transports (matching
    # run_calibration_icp.launch.py's defaults for the same bag,
    # bags/*_validation_lab) instead of the sim-flat names + a relay layer
    # to bridge them. image_transport=compressed here for the same "avoid
    # raw" reason threaded through calibration_icp_node and
    # onboard_detector_v2's preprocessing_node; depth_transport=zstd
    # because that's what this particular bag actually recorded — override
    # to compressedDepth for the live camera.
    declare_image_topic_cmd = DeclareLaunchArgument(
        'image_topic', default_value='/front_camera/camera/color/image_raw')
    declare_camera_info_topic_cmd = DeclareLaunchArgument(
        'camera_info_topic', default_value='/front_camera/camera/color/camera_info')
    declare_depth_topic_cmd = DeclareLaunchArgument(
        'depth_topic', default_value='/front_camera/camera/aligned_depth_to_color/image_raw')
    declare_image_transport_cmd = DeclareLaunchArgument(
        'image_transport', default_value='compressed')
    declare_depth_transport_cmd = DeclareLaunchArgument(
        'depth_transport', default_value='zstd')

    robot_localization_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[LaunchConfiguration('localization_params'), {'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )

    visodom = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_dir, 'launch', 'visodom.launch.py')
        ),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'image_topic': LaunchConfiguration('image_topic'),
            'camera_info_topic': LaunchConfiguration('camera_info_topic'),
            'depth_topic': LaunchConfiguration('depth_topic'),
            'image_transport': LaunchConfiguration('image_transport'),
            'depth_transport': LaunchConfiguration('depth_transport'),
        }.items(),
    )

    return LaunchDescription([
        declare_params_file_cmd,
        declare_use_sim_time_cmd,
        declare_image_topic_cmd,
        declare_camera_info_topic_cmd,
        declare_depth_topic_cmd,
        declare_image_transport_cmd,
        declare_depth_transport_cmd,
        visodom,
        robot_localization_node,
    ])
