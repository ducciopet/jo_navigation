#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    jo_nav_dir = get_package_share_directory('jo_navigation')
    jo_sim_dir = get_package_share_directory('jo_sim')

    glim_config = os.path.join(
        jo_sim_dir, 'config', 'glim', 'glim_config_bunker_sim'
    )

    # ── Arguments ──────────────────────────────────────────────────────────────
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='Use /clock — set true when replaying bags with --clock')

    glim_config_arg = DeclareLaunchArgument(
        'glim_config', default_value=glim_config,
        description='Path to GLIM config folder')

    # ── Relay bag topics to the names that visodom.launch.py expects ───────────
    # visodom reads: /front_camera/image, /front_camera/camera_info,
    #                /front_camera/depth_image  (then restamps to optical frame)
    relay_image = Node(
        package='topic_tools',
        executable='relay',
        name='relay_image',
        arguments=[
            '/front_camera/camera/color/image_raw',
            '/front_camera/image',
        ],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
    )

    relay_camera_info = Node(
        package='topic_tools',
        executable='relay',
        name='relay_camera_info',
        arguments=[
            '/front_camera/camera/color/camera_info',
            '/front_camera/camera_info',
        ],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
    )

    relay_depth = Node(
        package='topic_tools',
        executable='relay',
        name='relay_depth',
        arguments=[
            '/front_camera/camera/depth/image_rect_raw',
            '/front_camera/depth_image',
        ],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
    )

    # ── GLIM: LiDAR-IMU odometry ───────────────────────────────────────────────
    # lidar_dynamic_filter runs in passthrough mode until the detector is up,
    # then starts filtering dynamic clusters from /velodyne_points_filtered.
    glim_node = Node(
        package='glim_ros',
        executable='glim_rosnode',
        name='glim_ros',
        output='screen',
        emulate_tty=True,
        parameters=[
            {'config_path': LaunchConfiguration('glim_config')},
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ],
    )

    lidar_dynamic_filter = Node(
        package='jo_navigation',
        executable='lidar_dynamic_filter',
        output='screen',
        emulate_tty=True,
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
    )

    # ── Visual odometry (RTABMap) + EKF filter ─────────────────────────────────
    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(jo_nav_dir, 'launch', 'localization.launch.py')
        ),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }.items(),
    )

    return LaunchDescription([
        use_sim_time_arg,
        glim_config_arg,
        # Camera topic relay (immediate, before visodom needs them)
        relay_image,
        relay_camera_info,
        relay_depth,
        # LiDAR-IMU odometry
        glim_node,
        lidar_dynamic_filter,
        # Localization stack (visual odometry + EKF)
        localization,
    ])
