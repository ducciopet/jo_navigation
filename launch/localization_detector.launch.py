#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterFile


def _launch_setup(context, *args, **kwargs):
    jo_nav_dir = get_package_share_directory('jo_navigation')
    jo_sim_dir = get_package_share_directory('jo_sim')
    onboard_detector_dir = get_package_share_directory('onboard_detector')

    detector_config_file = os.path.join(
        onboard_detector_dir, 'cfg', 'detector_param_jo_zotac_indoor.yaml')

    depth_intrinsics_str = LaunchConfiguration('depth_intrinsics').perform(context)
    depth_intrinsics = [float(v) for v in depth_intrinsics_str.split(',')]
    if len(depth_intrinsics) != 4:
        raise ValueError(
            f"depth_intrinsics must be 4 comma-separated numbers [fx,fy,cx,cy], got: {depth_intrinsics_str!r}")

    # ── GLIM: LiDAR-IMU odometry ───────────────────────────────────────────
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

    # ── Lidar-camera extrinsic calibration (ICP, one-shot) ─────────────────
    # wall_detector_node's ground/wall pipeline is gated on the
    # calibration_child_frame TF (default "camera_refined", see its cfg)
    # actually being published, which only calibration_icp_node does —
    # without this node running, that gate would never open and wall
    # detection would silently never start. robot_tf_camera_frame uses the
    # bag's own recorded robot TF as the initial guess instead of a
    # hand-measured static one (see run_calibration_icp.launch.py, same
    # reasoning: the hardcoded guess is ~42cm off for this bag/robot).
    #
    # Gated on wall_detection too (same condition as wall_detector_node
    # below), NOT unconditional: this node's only reason to exist here is
    # feeding that node's TF gate — with wall_detection:=false there's
    # nothing left to consume "camera_refined" at all, so launching it
    # anyway just wastes a process and (worse) collides in NODE NAME with
    # onboard_detector_v2's own calibration_icp_node_<camera> instances
    # when this file runs alongside run_detector.launch.py — same problem
    # this file's own wall_detector_node has, see that Node's own comment.
    calibration_node = Node(
        package='onboard_detector',
        executable='calibration_icp_node',
        name='calibration_icp_node',
        output='screen',
        condition=IfCondition(LaunchConfiguration('wall_detection')),
        parameters=[
            ParameterFile(detector_config_file, allow_substs=True),
            {
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'depth_topic': LaunchConfiguration('depth_topic'),
                'depth_transport': LaunchConfiguration('depth_transport'),
                'onboard_detector.depth_intrinsics': depth_intrinsics,
                # depth_intrinsics above is now only calibration_icp_node's
                # bootstrap value — camera_info_topic (same topic visodom
                # already reads its own live intrinsics from, see
                # visodom.launch.py) makes it read the SAME color camera's
                # live K instead. See calibration_icp_node.cpp's own
                # onCameraInfo() comment.
                'onboard_detector.camera_info_topic': LaunchConfiguration('camera_info_topic'),
                'camera_frame_initial_guess': LaunchConfiguration('camera_frame_initial_guess'),
            },
        ],
    )

    # ── Wall/ground detection (LiDAR RANSAC + depth-camera ground/roof) ────
    # onboard_detector (v1) — deprecated in favor of onboard_detector_v2's
    # own static_structures_node (run_detector.launch.py). OFF by default now
    # (wall_detection:=false): with v2's pipeline running, a SAME-NAME
    # '/wall_detector_node' from each would collide in ros2 node list/
    # rqt_graph, and v1's would just duplicate v2's work. Same for
    # calibration_node above, which shares this condition.
    wall_detector_node = Node(
        package='onboard_detector',
        executable='wall_detector_node',
        name='wall_detector_node',
        output='screen',
        condition=IfCondition(LaunchConfiguration('wall_detection')),
        parameters=[
            ParameterFile(detector_config_file, allow_substs=True),
            {
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'wall_detector.depth_image_topic': LaunchConfiguration('depth_topic'),
                'wall_detector.depth_transport': LaunchConfiguration('depth_transport'),
                'wall_detector.depth_intrinsics': depth_intrinsics,
                # See calibration_node's own camera_info_topic comment above —
                # same reasoning, same topic.
                'wall_detector.camera_info_topic': LaunchConfiguration('camera_info_topic'),
            },
        ],
    )

    # ── Visual odometry (RTABMap) + EKF filter ──────────────────────────────
    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(jo_nav_dir, 'launch', 'localization.launch.py')
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

    # ── RViz: GLIM map + dynamic bbox visualization ─────────────────────────
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz_glim_bbox',
        output='screen',
        arguments=['-d', os.path.join(jo_sim_dir, 'rviz', 'glim_bbox.rviz')],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        condition=IfCondition(LaunchConfiguration('rviz')),
    )

    return [
        # LiDAR-IMU odometry
        glim_node,
        # Lidar-camera calibration (gates wall_detector_node's TF)
        calibration_node,
        # Wall/ground detection
        wall_detector_node,
        # Localization stack (visual odometry + EKF)
        localization,
        rviz,
    ]


def generate_launch_description():
    # ── Arguments ────────────────────────────────────────────────────────────
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='Use /clock — set true when replaying bags with --clock')

    glim_config_arg = DeclareLaunchArgument(
        'glim_config', default_value=os.path.join(
            get_package_share_directory('jo_sim'), 'config', 'glim', 'glim_config_bunker_sim'),
        description='Path to GLIM config folder')

    rviz_arg = DeclareLaunchArgument(
        'rviz', default_value='true',
        description='Whether to launch RViz with the GLIM bbox config')

    wall_detection_arg = DeclareLaunchArgument(
        'wall_detection', default_value='false',
        description="Whether to launch onboard_detector (v1)'s calibration_icp_node + "
                     'wall_detector_node (ground/wall detection). OFF by default: onboard_detector_v2 '
                     '(run_detector.launch.py) runs its own calibration_icp_node and static_structures_node, '
                     'so launching v1\'s would only duplicate them — see those two Nodes\' own '
                     'comments. Pass true only to run v1 alone (without run_detector.launch.py): v1\'s node NAMES collide '
                     "with v2's otherwise "
                     '(both are literally "/calibration_icp_node"/"/wall_detector_node", no '
                     'namespace), which rqt_graph and other introspection tools handle badly.')

    # Forwarded to localization.launch.py -> visodom.launch.py, and to
    # calibration_icp_node/wall_detector_node below. Defaults target
    # bags/*_validation_lab, which this launch file is for (default
    # use_sim_time=true, meant for bag replay): that bag only records
    # aligned-to-color depth as zstd and color as compressed — no raw
    # variant of either exists at all — so the old relay_image/
    # relay_camera_info/relay_depth trio that used to be here (topic_tools
    # relay from .../color/image_raw and .../depth/image_rect_raw, neither
    # of which exist in this bag) always sat there with no data; it's been
    # removed. Point these at a different bag/live camera by overriding all
    # of image_topic/camera_info_topic/depth_topic/image_transport/
    # depth_transport/depth_intrinsics together.
    image_topic_arg = DeclareLaunchArgument(
        'image_topic', default_value='/front_camera/camera/color/image_raw')
    camera_info_topic_arg = DeclareLaunchArgument(
        'camera_info_topic', default_value='/front_camera/camera/color/camera_info')
    depth_topic_arg = DeclareLaunchArgument(
        'depth_topic', default_value='/front_camera/camera/aligned_depth_to_color/image_raw')
    image_transport_arg = DeclareLaunchArgument(
        'image_transport', default_value='compressed')
    depth_transport_arg = DeclareLaunchArgument(
        'depth_transport', default_value='zstd')
    depth_intrinsics_arg = DeclareLaunchArgument(
        'depth_intrinsics', default_value='644.1800537109375,643.2573852539062,647.415283203125,361.88623046875',
        description='Comma-separated fx,fy,cx,cy for depth_topic — the COLOR camera intrinsics, since depth_topic '
                     'is an aligned_depth_to_color stream by default. Used by both calibration_icp_node and '
                     'wall_detector_node as their BOOTSTRAP value only — both now also subscribe to '
                     'camera_info_topic and overwrite fx/fy/cx/cy live from there the moment a message arrives '
                     '(see calibration_icp_node.cpp/wallDetector.cpp\'s own onCameraInfo/cameraInfoCallback).')
    camera_frame_initial_guess_arg = DeclareLaunchArgument(
        'camera_frame_initial_guess', default_value='front_camera_color_optical_frame',
        description="calibration_icp_node's initial-guess TF frame — see run_calibration_icp.launch.py's "
                     'robot_tf_camera_frame for the same reasoning (uses the bag/robot own TF tree instead of a '
                     'hand-measured guess).')

    return LaunchDescription([
        use_sim_time_arg,
        glim_config_arg,
        rviz_arg,
        wall_detection_arg,
        image_topic_arg,
        camera_info_topic_arg,
        depth_topic_arg,
        image_transport_arg,
        depth_transport_arg,
        depth_intrinsics_arg,
        camera_frame_initial_guess_arg,
        OpaqueFunction(function=_launch_setup),
    ])
