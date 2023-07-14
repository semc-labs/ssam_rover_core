import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import (ComposableNodeContainer, SetParameter,
                                SetParametersFromFile, SetRemap)
from launch_ros.descriptions import ComposableNode
from launch.actions import GroupAction



def generate_launch_description():

    bringup_dir = get_package_share_directory('nvblox_examples_bringup')

    # Nav2
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            bringup_dir, 'launch', 'nav2', 'nav2_isaac_sim.launch.py')),
        )

    # Nvblox
    base_config_dir = os.path.join(bringup_dir, 'config', 'nvblox')
    specialization_dir = os.path.join(base_config_dir, 'specializations')

    # Config files
    base_config = os.path.join(base_config_dir, 'nvblox_base.yaml')

    # Nvblox node
    node = ComposableNode(
        name='nvblox_node',
        package='nvblox_ros',
        plugin='nvblox::NvbloxNode')

    # Nvblox node container
    nvblox_container = ComposableNodeContainer(
        name='nvblox_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[node],
        output='screen')

    nvblox_launch_node = GroupAction([

        # Set parameters with specializations
        SetParametersFromFile(base_config),

        SetParameter(name='global_frame',
                     value=LaunchConfiguration('global_frame', default='odom')),

        # Remappings for realsense data
        SetRemap(src=['depth/image'],
                 dst=['/depth_camera/depth/image_raw']),
        SetRemap(src=['depth/camera_info'],
                 dst=['/depth_camera/depth/camera_info']),
        SetRemap(src=['color/image'],
                 dst=['/left_camera/image_raw']),
        SetRemap(src=['color/camera_info'],
                 dst=['/left_camera/camera_info']),

        # Include the node container
        nvblox_container
    ])

    return LaunchDescription([
        nav2_launch,
        nvblox_launch_node])
