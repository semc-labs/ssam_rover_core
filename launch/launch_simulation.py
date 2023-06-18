import os

from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessStart, OnProcessExit, OnExecutionComplete
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch.actions import DeclareLaunchArgument, TimerAction

from launch_ros.actions import Node

# import xacro


def generate_launch_description():

    use_ros2_control = LaunchConfiguration('use_ros2_control')

    world_path = os.path.join(get_package_share_directory('ssam_rover_core'), 'worlds', 'room.world')
    gazebo_params_file = os.path.join(get_package_share_directory('ssam_rover_core'), 'config', 'gazebo_params.yaml')
    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(get_package_share_directory('gazebo_ros'), 'launch'), '/gazebo.launch.py']),
                launch_arguments={'params_file': gazebo_params_file, 'world': world_path}.items()
    )

    pkg_path = os.path.join(get_package_share_directory('ssam_rover_core'))
    xacro_file = os.path.join(pkg_path,'description','robot.urdf.xacro')    
    robot_description_config = Command(['xacro ', xacro_file, ' use_ros2_control:=', use_ros2_control])

    params = {'robot_description': robot_description_config, 'use_sim_time': True}

    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )

    spawn_entity = ExecuteProcess(
        cmd=['ros2', 'run', 'gazebo_ros', 'spawn_entity.py', '-topic', 'robot_description', '-entity', 'diffbot'], output='screen'
    )

    diff_drive_spawner = ExecuteProcess(
        cmd=['ros2', 'run', 'controller_manager', 'spawner', 'diff_cont']
    )

    joint_broad_spawner = ExecuteProcess(
        cmd=['ros2', 'run', 'controller_manager', 'spawner', 'joint_broad']
    )

    slam_launch = ExecuteProcess(
        cmd=['ros2', 'launch', 'ssam_rover_core', 'launch_lidar_slam.py'], output='screen'
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_ros2_control', default_value='true', description='Use ros2 control'),

        node_robot_state_publisher,
        gazebo,
        spawn_entity,
        joint_broad_spawner,
        diff_drive_spawner,
        TimerAction(period=3.0, actions=[slam_launch])
    ])