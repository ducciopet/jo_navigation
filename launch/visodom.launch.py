from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    config = os.path.join(
        get_package_share_directory('jo_navigation'),
        'config',
        'visual_odom.yaml'
    )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description='Use simulation clock')

    repub_image = Node(
        package='topic_tools',
        executable='transform',
        name='image_restamper',
        arguments=[
            '/front_camera/image',
            '/front_camera/image_optical',
            'sensor_msgs/msg/Image',
            'setattr(m.header, "frame_id", "front_camera_optical_frame") or m',
        ]
    )

    repub_camera_info = Node(
        package='topic_tools',
        executable='transform',
        name='camera_info_restamper',
        arguments=[
            '/front_camera/camera_info',
            '/front_camera/camera_info_optical',
            'sensor_msgs/msg/CameraInfo',
            'setattr(m.header, "frame_id", "front_camera_optical_frame") or m',
        ]
    )

    repub_depth = Node(
        package='topic_tools',
        executable='transform',
        name='depth_restamper',
        arguments=[
            '/front_camera/depth_image',
            '/front_camera/depth_image_optical',
            'sensor_msgs/msg/Image',
            'setattr(m.header, "frame_id", "front_camera_optical_frame") or m',
        ]
    )

    rgbd_odometry = Node(
        package='rtabmap_odom',
        executable='rgbd_odometry',
        name='rgbd_odometry',
        namespace='visodom',
        output='log',
        parameters=[config, {'use_sim_time': LaunchConfiguration('use_sim_time')}],
        remappings=[
            ('rgb/image',       '/front_camera/image_optical'),
            ('rgb/camera_info', '/front_camera/camera_info_optical'),
            ('depth/image',     '/front_camera/depth_image_optical'),
            ('scan_cloud',      '/front_camera/camera/depth/color/points'),
        ],
    )

    rtabmap = Node(
        package='rtabmap_slam',
        executable='rtabmap',
        name='rtabmap',
        namespace='visodom',
        output='screen',
        parameters=[config],
        remappings=[
            ('rgb/image',       '/front_camera/camera/color/image_raw'),
            ('rgb/camera_info', '/front_camera/camera/color/camera_info'),
            ('depth/image',     '/front_camera/camera/aligned_depth_to_color/image_raw'),
            ('scan_cloud',      '/front_camera/camera/depth/color/points'),
        ],
    )

    rtabmap_viz = Node(
        package='rtabmap_viz',
        executable='rtabmap_viz',
        name='rtabmap_viz',
        namespace='visodom',
        output='screen',
        parameters=[config],
        remappings=[
            ('rgb/image',       '/front_camera/camera/color/image_raw'),
            ('rgb/camera_info', '/front_camera/camera/color/camera_info'),
            ('depth/image',     '/front_camera/camera/aligned_depth_to_color/image_raw'),
            ('scan_cloud',      '/front_camera/camera/depth/color/points'),
        ],
    )

    return LaunchDescription([
        use_sim_time_arg,
        repub_image,
        repub_camera_info,
        repub_depth,
        rgbd_odometry,
        # rtabmap,
        # rtabmap_viz,
    ])