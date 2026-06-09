"""
04_coverage_clean.launch.py
===========================
Coverage cleaning pipeline:
  1. Nav2 full stack (with saved map + AMCL)
  2. Coverage Server (opennav_coverage)
  3. Map-to-Polygon conversion node
  4. Coverage Client node (auto-starts cleaning)

Usage:
    export TURTLEBOT3_MODEL=waffle
    ros2 launch clean_robot_pkg 04_coverage_clean.launch.py

    # Or with custom map:
    ros2 launch clean_robot_pkg 04_coverage_clean.launch.py map:=/path/to/map.yaml
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_clean_robot = get_package_share_directory('clean_robot_pkg')
    pkg_turtlebot3_gazebo = get_package_share_directory('turtlebot3_gazebo')

    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    map_file = LaunchConfiguration(
        'map',
        default=os.path.join(pkg_clean_robot, 'maps', 'house_map.yaml'),
    )
    nav2_params_file = os.path.join(pkg_clean_robot, 'config', 'nav2_params.yaml')
    coverage_params_file = os.path.join(pkg_clean_robot, 'config', 'coverage_params.yaml')

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

    # --- Nav2 Core Nodes ---
    controller_server_node = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[nav2_params_file, {'use_sim_time': use_sim_time}],
    )

    planner_server_node = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[nav2_params_file, {'use_sim_time': use_sim_time}],
    )

    behavior_server_node = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        output='screen',
        parameters=[nav2_params_file, {'use_sim_time': use_sim_time}],
    )

    bt_navigator_node = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[nav2_params_file, {'use_sim_time': use_sim_time}],
    )

    # --- Lifecycle Manager for Nav2 ---
    lifecycle_manager_nav = Node(
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

    # --- Coverage Server (opennav_coverage) ---
    coverage_server_node = Node(
        package='opennav_coverage',
        executable='coverage_server',
        name='coverage_server',
        output='screen',
        parameters=[
            coverage_params_file,
            {'use_sim_time': use_sim_time},
        ],
    )

    # --- Lifecycle Manager for Coverage ---
    lifecycle_manager_coverage = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_coverage',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'autostart': True},
            {'node_names': ['coverage_server']},
        ],
    )

    # --- Map to Polygon Converter ---
    map_to_polygon_node = Node(
        package='clean_robot_pkg',
        executable='map_to_polygon_node.py',
        name='map_to_polygon_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    # --- Coverage Client (auto-start cleaning after delay) ---
    # Delayed start to wait for Nav2 + Coverage server to be ready
    coverage_client_node = TimerAction(
        period=15.0,  # Wait 15 seconds for all servers to come up
        actions=[
            Node(
                package='clean_robot_pkg',
                executable='coverage_client_node.py',
                name='coverage_client_node',
                output='screen',
                parameters=[{'use_sim_time': use_sim_time}],
            ),
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
        # Core
        gazebo_launch,
        # Nav2
        map_server_node,
        amcl_node,
        controller_server_node,
        planner_server_node,
        behavior_server_node,
        bt_navigator_node,
        lifecycle_manager_nav,
        # Coverage
        coverage_server_node,
        lifecycle_manager_coverage,
        map_to_polygon_node,
        coverage_client_node,
        # Visualization
        rviz_node,
    ])
