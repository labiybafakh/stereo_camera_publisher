#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get the package directory
    package_dir = get_package_share_directory('stereo_camera_publisher')

    # Path to the config file
    config_file = os.path.join(package_dir, 'config', 'stereo_camera_params.yaml')

    # Declare launch argument for config file path (optional override)
    config_arg = DeclareLaunchArgument(
        'config_file',
        default_value=config_file,
        description='Path to the configuration file'
    )

    # Create the stereo camera node
    stereo_camera_node = Node(
        package='stereo_camera_publisher',
        executable='stereo_camera_node',
        name='stereo_camera_publisher',
        output='screen',
        parameters=[LaunchConfiguration('config_file')]
    )

    return LaunchDescription([
        config_arg,
        stereo_camera_node
    ])
