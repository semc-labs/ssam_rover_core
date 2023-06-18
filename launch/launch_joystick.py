import os

from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch.actions import DeclareLaunchArgument

from launch_ros.actions import Node

# import xacro


def generate_launch_description():

    params_file = os.path.join(get_package_share_directory('ssam_rover_core'), 'config', 'joystick.yaml')

    joy_node = Node(
        package='joy',
        executable='joy_node',
        parameters=[params_file]
    )

    teleop_node = Node(package="teleop_twist_joy",
            executable="teleop_node",
            name="teleop_joy",
            parameters=[params_file],
            remappings=[('/cmd_vel', '/cmd_vel_joy')])

    return LaunchDescription([
        joy_node,
        teleop_node
    ])