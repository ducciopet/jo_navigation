import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import TimerAction, ExecuteProcess


def generate_launch_description():
    # Get the launch directory
    pkg_dir = get_package_share_directory('jo_navigation')


    declare_params_file_cmd = DeclareLaunchArgument(
        'localization_params',
        default_value=os.path.join(pkg_dir, 'config', 'localization_gps.yaml'),
        description='Full path to the ROS2 parameters file to use for all launched nodes')
    
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true')
 
    robot_localization_global = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node_map",
        output="screen",
        remappings=[("odometry/filtered", "odometry/global")],
        parameters=[LaunchConfiguration('localization_params'), {'use_sim_time': LaunchConfiguration('use_sim_time')}],
    )

    robot_localization_local = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node_odom",
        output="screen",
        remappings=[("odometry/filtered", "odometry/local")],
        parameters=[LaunchConfiguration('localization_params'), {'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )


    navsat_transform_node = Node(
        package='robot_localization',
        executable='navsat_transform_node',
        name='navsat_transform_node',
        output='log',
        parameters=[LaunchConfiguration('localization_params')],
        arguments=['--ros-args','--log-level','WARN'],
        remappings=[
            ('imu', 'imu/data'),
            ('gps/fix', 'gnss'),      # adjust
            ('odometry/filtered', 'odometry/global'), # from local EKF
        ]
    )


    set_zone = TimerAction(
        period=1.0,
        actions=[
            ExecuteProcess(
            cmd=[
                'ros2', 'service', 'call', '/setUTMZone',
                'robot_localization/srv/SetUTMZone',
                '{utm_zone: 32N}'
            ],
            output='screen'
            )
        ]
    )
   


    return LaunchDescription([
        declare_params_file_cmd,
        declare_use_sim_time_cmd,
        robot_localization_global,
        robot_localization_local,
        navsat_transform_node,
        set_zone,
    ])
