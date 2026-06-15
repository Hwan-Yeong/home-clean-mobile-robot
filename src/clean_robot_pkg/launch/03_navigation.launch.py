"""
03_navigation.launch.py
=======================
Full Nav2 stack with a pre-saved map.
Starts: Gazebo + AMCL + Nav2 (controller, planner, behavior, bt_navigator) + RViz

Usage:
    export TURTLEBOT3_MODEL=waffle
    ros2 launch clean_robot_pkg 03_navigation.launch.py

    # Or with custom map:
    ros2 launch clean_robot_pkg 03_navigation.launch.py map:=/path/to/map.yaml
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_clean_robot = get_package_share_directory('clean_robot_pkg')
    pkg_turtlebot3_gazebo = get_package_share_directory('turtlebot3_gazebo')

    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    # Use src directory to ensure map availability
    maps_dir = '/home/hyjoe/hyjoe_repositories/src/home-clean-mobile-robot/src/clean_robot_pkg/maps'
    map_file = LaunchConfiguration(
        'map',
        default=os.path.join(maps_dir, 'house_map.yaml'),
    )
    nav2_params_file = LaunchConfiguration(
        'params_file',
        default=os.path.join(pkg_clean_robot, 'config', 'nav2_params.yaml'),
    )

    # --- Gazebo: TurtleBot3 House World ---
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_turtlebot3_gazebo, 'launch', 'turtlebot3_house.launch.py')
        ),
    )

    # --- Map Server ---
    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'yaml_filename': map_file},
        ],
    )

    # --- AMCL ---
    amcl_node = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[
            nav2_params_file,
            {'use_sim_time': use_sim_time},
        ],
    )

    # --- Controller Server ---
    controller_server_node = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[
            nav2_params_file,
            {'use_sim_time': use_sim_time},
        ],
    )

    # --- Planner Server ---
    planner_server_node = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[
            nav2_params_file,
            {'use_sim_time': use_sim_time},
        ],
    )

    # --- Behavior Server ---
    behavior_server_node = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        output='screen',
        parameters=[
            nav2_params_file,
            {'use_sim_time': use_sim_time},
        ],
    )

    # --- BT Navigator ---
    bt_navigator_node = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[
            nav2_params_file,
            {'use_sim_time': use_sim_time},
        ],
    )

    # --- Lifecycle Manager ---
    lifecycle_manager_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'autostart': True},
            {'node_names': [
                'map_server',
                'amcl',
                'controller_server',
                'planner_server',
                'behavior_server',
                'bt_navigator',
            ]},
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
            'use_sim_time', default_value='true',
            description='Use simulation clock',
        ),
        DeclareLaunchArgument(
            'map',
            default_value=os.path.join(pkg_clean_robot, 'maps', 'house_map.yaml'),
            description='Full path to the map yaml file',
        ),
        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(pkg_clean_robot, 'config', 'nav2_params.yaml'),
            description='Full path to the Nav2 params file',
        ),
        gazebo_launch,
        map_server_node,
        amcl_node,
        controller_server_node,
        planner_server_node,
        behavior_server_node,
        bt_navigator_node,
        lifecycle_manager_node,
        rviz_node,
    ])
