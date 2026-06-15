"""
02_save_map.launch.py
=====================
Save the current SLAM map to a file.
This launch file calls nav2_map_server's map_saver_cli.

Usage:
    ros2 launch clean_robot_pkg 02_save_map.launch.py

    # Or with custom map name:
    ros2 launch clean_robot_pkg 02_save_map.launch.py map_name:=my_house_map
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # Map directory in the source tree to avoid data loss during fresh builds
    maps_dir = '/home/hyjoe/hyjoe_repositories/src/home-clean-mobile-robot/src/clean_robot_pkg/maps'

    map_name = LaunchConfiguration('map_name', default='house_map')

    # Save map using nav2_map_server CLI
    save_map_cmd = ExecuteProcess(
        cmd=[
            'ros2', 'run', 'nav2_map_server', 'map_saver_cli',
            '-f', [maps_dir, '/', map_name],
            '--ros-args', '-p', 'use_sim_time:=true',
        ],
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'map_name',
            default_value='house_map',
            description='Name of the map file to save (without extension)',
        ),
        save_map_cmd,
    ])
