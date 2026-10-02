from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def _launch_setup(context, *args, **kwargs):
    config = os.path.join(
        get_package_share_directory('jo_navigation'),
        'config',
        'visual_odom.yaml'
    )

    image_topic = LaunchConfiguration('image_topic').perform(context)
    camera_info_topic = LaunchConfiguration('camera_info_topic').perform(context)
    depth_topic = LaunchConfiguration('depth_topic').perform(context)
    image_transport = LaunchConfiguration('image_transport').perform(context)
    depth_transport = LaunchConfiguration('depth_transport').perform(context)

    # The 'raw' transport is what the simulated camera (gz_bridge.yaml) and
    # jo_sim's image_bridge actually publish: plain sensor_msgs/Image on the
    # exact topic names below, with a frame_id that doesn't match the URDF's
    # optical frame, hence the restamper trio. A recorded bag (e.g.
    # bags/*_validation_lab) is different on both counts: it only has
    # compressed/zstd image_transport topics (no raw variant at all — see
    # run_calibration_icp.launch.py's own depth_topic/depth_transport args
    # for the same situation on the calibration side), and its recorded
    # header.frame_id is already the correct URDF optical frame
    # (front_camera_color_optical_frame), so no restamping is needed or even
    # possible (topic_tools transform/relay can't decode a compressed
    # message — it needs the exact message type up front). So: restamp only
    # for the raw/sim case; for anything else, remap rgbd_odometry straight
    # onto the given topics and let it use image_transport itself.
    is_raw = (image_transport in ('', 'raw')) and (depth_transport in ('', 'raw'))

    actions = []

    if is_raw:
        actions.append(Node(
            package='topic_tools',
            executable='transform',
            name='image_restamper',
            arguments=[
                image_topic,
                '/front_camera/image_optical',
                'sensor_msgs/msg/Image',
                'setattr(m.header, "frame_id", "front_camera_optical_frame") or m',
            ]
        ))
        actions.append(Node(
            package='topic_tools',
            executable='transform',
            name='camera_info_restamper',
            arguments=[
                camera_info_topic,
                '/front_camera/camera_info_optical',
                'sensor_msgs/msg/CameraInfo',
                'setattr(m.header, "frame_id", "front_camera_optical_frame") or m',
            ]
        ))
        actions.append(Node(
            package='topic_tools',
            executable='transform',
            name='depth_restamper',
            arguments=[
                depth_topic,
                '/front_camera/depth_image_optical',
                'sensor_msgs/msg/Image',
                'setattr(m.header, "frame_id", "front_camera_optical_frame") or m',
            ]
        ))
        rgbd_odometry_params = [config, {'use_sim_time': LaunchConfiguration('use_sim_time')}]
        rgbd_odometry_remappings = [
            ('rgb/image',       '/front_camera/image_optical'),
            ('rgb/camera_info', '/front_camera/camera_info_optical'),
            ('depth/image',     '/front_camera/depth_image_optical'),
            ('scan_cloud',      '/front_camera/camera/depth/color/points'),
        ]
    else:
        # header.frame_id is already correct (assumed above), but
        # header.stamp is NOT: at least the zstd_image_transport subscriber
        # plugin decodes to stamp=(0,0), and rgbd_odometry's own
        # message_filters::Synchronizer<ApproximateTime<...>> compares
        # *decoded* stamps — with depth stuck at zero against color/
        # camera_info's real stamps, it would never find a match at any
        # tolerance (confirmed: even an unbounded approx_sync_max_interval
        # doesn't help, since a 0 vs ~1.7e9 gap swamps any finite window).
        # image_restamp_node decodes once via image_transport and
        # republishes raw with header.stamp = this->now() (which, under
        # use_sim_time, tracks /clock — i.e. the bag's own timeline, not
        # wall-clock — so it stays close to the still-correct camera_info
        # stamp). rgbd_odometry then subscribes those restamped topics with
        # transport 'raw', where the whole problem doesn't apply.
        color_restamped_topic = '/visodom/color_restamped'
        depth_restamped_topic = '/visodom/depth_restamped'

        actions.append(Node(
            package='onboard_detector',
            executable='image_restamp_node',
            name='color_restamp_node',
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'input_topic': image_topic,
                'input_transport': image_transport,
                'output_topic': color_restamped_topic,
            }],
        ))
        actions.append(Node(
            package='onboard_detector',
            executable='image_restamp_node',
            name='depth_restamp_node',
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'input_topic': depth_topic,
                'input_transport': depth_transport,
                'output_topic': depth_restamped_topic,
            }],
        ))

        rgbd_odometry_params = [
            config,
            {
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'image_transport': 'raw',
                'depth_transport': 'raw',
                # Two independent restamp nodes each timestamp with their
                # own this->now() call, adding latency/scheduling skew on
                # top of the two frames' original (tiny) offset — loosen
                # the yaml's tight 0.02s indoor-camera-pair tolerance to
                # match calibration_icp_node's own proven margin for this
                # same restamp pattern (kMaxSyncDeltaSec = 0.15s there).
                'approx_sync_max_interval': 0.15,
            },
        ]
        rgbd_odometry_remappings = [
            ('rgb/image',       color_restamped_topic),
            ('rgb/camera_info', camera_info_topic),
            ('depth/image',     depth_restamped_topic),
            ('scan_cloud',      '/front_camera/camera/depth/color/points'),
        ]

    rgbd_odometry = Node(
        package='rtabmap_odom',
        executable='rgbd_odometry',
        name='rgbd_odometry',
        namespace='visodom',
        output='log',
        parameters=rgbd_odometry_params,
        remappings=rgbd_odometry_remappings,
    )
    actions.append(rgbd_odometry)

    return actions


def generate_launch_description():
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description='Use simulation clock')

    # Defaults match jo_sim's gz_bridge.yaml / image_bridge (raw sim topics).
    # For a recorded bag with only compressed/zstd streams (e.g.
    # bags/*_validation_lab), override e.g.:
    #   image_topic:=/front_camera/camera/color/image_raw
    #   camera_info_topic:=/front_camera/camera/color/camera_info
    #   depth_topic:=/front_camera/camera/aligned_depth_to_color/image_raw
    #   image_transport:=compressed  depth_transport:=zstd
    image_topic_arg = DeclareLaunchArgument(
        'image_topic', default_value='/front_camera/image',
        description='Color image base topic (image_transport appends the transport-specific suffix)')
    camera_info_topic_arg = DeclareLaunchArgument(
        'camera_info_topic', default_value='/front_camera/camera_info',
        description='CameraInfo topic matching image_topic (never compressed, no suffix)')
    depth_topic_arg = DeclareLaunchArgument(
        'depth_topic', default_value='/front_camera/depth_image',
        description='Depth image base topic (image_transport appends the transport-specific suffix)')
    image_transport_arg = DeclareLaunchArgument(
        'image_transport', default_value='raw',
        description="image_transport plugin for image_topic: 'raw', 'compressed', 'zstd', ...")
    depth_transport_arg = DeclareLaunchArgument(
        'depth_transport', default_value='raw',
        description="image_transport plugin for depth_topic: 'raw', 'compressedDepth', 'zstd', ...")

    return LaunchDescription([
        use_sim_time_arg,
        image_topic_arg,
        camera_info_topic_arg,
        depth_topic_arg,
        image_transport_arg,
        depth_transport_arg,
        OpaqueFunction(function=_launch_setup),
    ])

# NOTE: rtabmap/rtabmap_viz (SLAM + its viewer, as opposed to rgbd_odometry
# above) used to be defined here too but were never actually launched (left
# commented out of the returned LaunchDescription) and referenced topics
# that don't exist as raw streams in bags/*_validation_lab either
# (/front_camera/camera/color/image_raw has no raw variant, only
# .../compressed — see the image_topic/image_transport args above). Removed
# rather than fixed since nothing enables them; reintroduce with the same
# image_topic/camera_info_topic/depth_topic/image_transport/depth_transport
# args as rgbd_odometry if/when they're needed.