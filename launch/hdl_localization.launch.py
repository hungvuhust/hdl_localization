#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Get package directory
    pkg_dir = get_package_share_directory('hdl_localization')

    # Parameter file path
    params_file = os.path.join(
        pkg_dir, 'config', 'hdl_localization_params.yaml')

    # Launch the main executable
    hdl_localization_main = Node(
        package='hdl_localization',
        executable='hdl_localization_main',
        parameters=[params_file],
        output='screen'
    )

    return LaunchDescription([
        hdl_localization_main
    ])
