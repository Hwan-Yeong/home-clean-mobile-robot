"""
01_gazebo_slam.launch.py
========================
Gazebo House World + TurtleBot3 Waffle + SLAM Toolbox + RViz

Usage:
    export TURTLEBOT3_MODEL=waffle
    ros2 launch clean_robot_pkg 01_gazebo_slam.launch.py

After launch, open a new terminal and run teleop:
    ros2 run teleop_twist_keyboard teleop_twist_keyboard
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    ExecuteProcess,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Package directories
    pkg_clean_robot = get_package_share_directory('clean_robot_pkg')
    pkg_turtlebot3_gazebo = get_package_share_directory('turtlebot3_gazebo')

    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # --- Gazebo: TurtleBot3 House World ---
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_turtlebot3_gazebo, 'launch', 'turtlebot3_house.launch.py')
        ),
    )

    # --- SLAM Toolbox (online async) ---
    slam_params_file = os.path.join(pkg_clean_robot, 'config', 'slam_params.yaml')

    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            slam_params_file,
            {'use_sim_time': use_sim_time},
        ],
    )

    # --- RViz2 ---
    rviz_config_file = os.path.join(pkg_clean_robot, 'rviz', 'clean_robot.rviz')

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock',
        ),
        gazebo_launch,
        slam_toolbox_node,
        rviz_node,
    ])
