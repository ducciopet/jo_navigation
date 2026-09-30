import os
import re
import shutil

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def glim_config_path(context, glim_config):
    """GLIM config dir to use; without the standard viewer if glim_view is false.

    The viewer (libstandard_viewer.so) needs a display: if the X connection to
    the remote PC drops, it closes and takes GLIM down with it. GLIM has no
    parameter to skip an extension module, so for glim_view:=false we copy the
    config dir to /tmp and remove the module from the copy's config_ros.json.
    """
    if LaunchConfiguration('glim_view').perform(context).lower() in ('true', '1'):
        return glim_config
    headless = '/tmp/glim_config_headless'
    shutil.rmtree(headless, ignore_errors=True)
    shutil.copytree(glim_config, headless)
    ros_json = os.path.join(headless, 'config_ros.json')
    with open(ros_json) as f:
        text = f.read()
    # Drop the entry together with the comma that separates it from a neighbour
    stripped = re.sub(r'"libstandard_viewer\.so"\s*,', '', text, count=1)
    if stripped == text:
        stripped = re.sub(r',(\s*)"libstandard_viewer\.so"', r'\1', text, count=1)
    with open(ros_json, 'w') as f:
        f.write(stripped)
    return headless


def generate_launch_description():
    # Get the launch directory
    pkg_dir = get_package_share_directory('jo_navigation')

    # LAUNCH PARAMS
    launch_glim_arg = DeclareLaunchArgument(
        'glim',
        default_value='true',
        description='Whether to launch the GLIM stack'
    )


    launch_glim_view_arg = DeclareLaunchArgument(
        'glim_view',
        default_value='false',
        description='Whether to open the GLIM viewer window (needs a display; GLIM dies if it is lost)'
    )


    launch_visodom_arg = DeclareLaunchArgument(
        'visodom',
        default_value='true',
        description='Whether to launch the visual odometry nodes'
    )


    launch_vslam_arg = DeclareLaunchArgument(
        'vslam',
        default_value='false',
        description='Whether to launch RTAB-Map visual SLAM (mapping mode)'
    )


    declare_params_file_cmd = DeclareLaunchArgument(
        'localization_params',
        default_value=os.path.join(pkg_dir, 'config', 'localization.yaml'),
        description='Full path to the ROS2 parameters file to use for all launched nodes')
    
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true')
    


    # LAUNCH FILES
    visodom_launch = os.path.join(pkg_dir, 'launch', 'visual_odom.launch.py')


    # CONFIG FILES
    glim_config = os.path.join(pkg_dir, 'config', 'glim', 'glim_config_bunker')



    # INCLUDED LAUNCH FILES

    visodom = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(visodom_launch),          
        launch_arguments={'vslam': LaunchConfiguration('vslam')}.items(),
        condition=IfCondition(LaunchConfiguration('visodom'))
    )   




    # NODES
    def launch_glim(context):
        return [Node(
            package='glim_ros',
            executable='glim_rosnode',
            output='screen',
            emulate_tty=True,
            additional_env={
                '__NV_PRIME_RENDER_OFFLOAD': '0',
            },
            parameters=[
                {'config_path': glim_config_path(context, glim_config)},
                ],
        )]

    glim = OpaqueFunction(
        function=launch_glim,
        condition=IfCondition(LaunchConfiguration('glim')),
    )
 

    robot_localization_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[LaunchConfiguration('localization_params'), {'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )
    


    return LaunchDescription([
        launch_glim_arg,
        launch_glim_view_arg,
        launch_visodom_arg,
        launch_vslam_arg,
        declare_params_file_cmd,
        declare_use_sim_time_cmd,
        robot_localization_node,
        glim,
        visodom
    ])
