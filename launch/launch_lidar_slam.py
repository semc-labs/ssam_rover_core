import os

from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch.actions import DeclareLaunchArgument, GroupAction, TimerAction

from launch_ros.actions import Node, SetRemap

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')

    slam_params_file = os.path.join(get_package_share_directory('ssam_rover_core'), 'config', 'lidar_slam_params.yaml')

    slam_node = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(get_package_share_directory('slam_toolbox'), 'launch'), '/online_async_launch.py']),
                launch_arguments={'slam_params_file': slam_params_file, 'use_sim_time': use_sim_time}.items()
    )

    navigation_node = GroupAction(
        actions=[
                SetRemap('/cmd_vel', '/diff_cont/cmd_vel_unstamped'),
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource([os.path.join(get_package_share_directory('nav2_bringup'), 'launch'), '/navigation_launch.py']),
                    launch_arguments={'use_sim_time': use_sim_time}.items()
                )
        ]
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true', description='Use sim time'),
        slam_node,
        navigation_node
    ])