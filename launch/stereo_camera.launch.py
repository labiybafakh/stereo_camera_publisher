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

    # Declare launch arguments
    config_arg = DeclareLaunchArgument(
        'config_file',
        default_value=config_file,
        description='Path to the configuration file'
    )

    image_width_arg = DeclareLaunchArgument(
        'image_width',
        default_value='1280',
        description='Image width in pixels'
    )

    image_height_arg = DeclareLaunchArgument(
        'image_height',
        default_value='720',
        description='Image height in pixels'
    )

    frame_rate_arg = DeclareLaunchArgument(
        'frame_rate',
        default_value='60.0',
        description='Frame rate in Hz'
    )

    exposure_arg = DeclareLaunchArgument(
        'exposure',
        default_value='3000',
        description='Exposure value (1-10000, higher=brighter)'
    )

    gain_arg = DeclareLaunchArgument(
        'gain',
        default_value='120',
        description='Gain value (0-255, higher=brighter)'
    )

    # Create the stereo camera node
    stereo_camera_node = Node(
        package='stereo_camera_publisher',
        executable='stereo_camera_node',
        name='stereo_camera_publisher',
        output='screen',
        parameters=[
            LaunchConfiguration('config_file'),
            {
                'image_width': LaunchConfiguration('image_width'),
                'image_height': LaunchConfiguration('image_height'),
                'frame_rate': LaunchConfiguration('frame_rate'),
                'exposure': LaunchConfiguration('exposure'),
                'gain': LaunchConfiguration('gain'),
            }
        ]
    )

    return LaunchDescription([
        config_arg,
        image_width_arg,
        image_height_arg,
        frame_rate_arg,
        exposure_arg,
        gain_arg,
        stereo_camera_node
    ])
