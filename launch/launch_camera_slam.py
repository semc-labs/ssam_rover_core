import launch
import os

from ament_index_python.packages import get_package_share_directory

from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    """Launch file to bring up visual slam node standalone."""
    visual_slam_node = ComposableNode(
        name='visual_slam_node',
        package='isaac_ros_visual_slam',
        plugin='isaac_ros::visual_slam::VisualSlamNode',
        remappings=[('/stereo_camera/left/camera_info', '/left_camera/camera_info'),
                    ('/stereo_camera/right/camera_info', '/right_camera/camera_info'),
                    ('/stereo_camera/left/image', '/left_camera/image_raw'),
                    ('/stereo_camera/right/image', '/right_camera/image_raw'),
                    ('/visual_slam/imu', '/realsense_imu/out')],
        parameters=[{
                    'use_sim_time': True,
                    'denoise_input_images': True,
                    'rectified_images': True,
                    'enable_imu_fusion': True,
                    'publish_odom_to_base_tf': False,
                    'enable_slam_visualization': True,
                    'enable_observations_view': True,
                    'enable_landmarks_view': True,
                    'enable_debug_mode': False,
                    'debug_dump_path': '/tmp/cuvslam',
                    'map_frame': 'map',
                    'odom_frame': 'odom',
                    'base_frame': 'base_link',
                    'input_left_camera_frame': 'left_camera_link',
                    'input_right_camera_frame': 'right_camera_link',
                    'input_imu_frame': 'imu_link',
                    }]
    )

    visual_slam_launch_container = ComposableNodeContainer(
        name='visual_slam_launch_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            visual_slam_node
        ],
        output='screen'
    )

    nvblox = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(get_package_share_directory('ssam_rover_core'), 'launch'), '/launch_nvblox.py'])    
    )

    return launch.LaunchDescription([
        visual_slam_launch_container,
        nvblox
    ])
