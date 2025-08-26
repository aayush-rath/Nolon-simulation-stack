"""
Launch file for Navigation2 with AMCL and RViz visualization - IMPROVED VERSION
Author: Aayush Rath | Date: 25/08/2025

This launch file starts:
- Map Server (loads prebuilt map)
- AMCL for localization
- Nav2 Planner, Controller, Smoother, BT Navigator
- Waypoint Follower (optional)
- Lifecycle Manager to control Nav2 nodes
- RViz with navigation configuration
- QoS overrides and logging improvements
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    map_yaml_file = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')
    cmd_vel_out = LaunchConfiguration('cmd_vel_out')

    # Paths
    pkg_share = FindPackageShare('nolon_bot_description')
    default_map = PathJoinSubstitution([pkg_share, 'maps', 'map.yaml'])
    default_params = PathJoinSubstitution([pkg_share, 'config', 'nolon_bot_navigation.yaml'])
    default_rviz = PathJoinSubstitution([pkg_share, 'rviz', 'nolon_bot_navigation.rviz'])

    return LaunchDescription([
        # Declare launch arguments
        DeclareLaunchArgument(
            'use_sim_time', default_value='true',
            description='Use simulation (Gazebo) clock'
        ),
        DeclareLaunchArgument(
            'map', default_value=default_map,
            description='Full path to map yaml file'
        ),
        DeclareLaunchArgument(
            'params_file', default_value=default_params,
            description='Full path to the ROS2 parameters file to use'
        ),
        DeclareLaunchArgument(
            'cmd_vel_out', default_value='/mobile_base/cmd_vel',
            description='Cmd vel output topic to your diff-drive controller'
        ),

        # Map Server
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time},
                        {'yaml_filename': map_yaml_file}],
        ),

        # AMCL
        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[params_file],
            remappings=[('scan', '/scan')],
        ),

        # Planner
        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=[params_file],
        ),

        # Controller
        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            output='screen',
            parameters=[params_file],
            remappings=[('cmd_vel', cmd_vel_out)],
        ),

        # Smoother
        Node(
            package='nav2_smoother',
            executable='smoother_server',
            name='smoother_server',
            output='screen',
            parameters=[params_file],
        ),

        # BT Navigator
        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            parameters=[params_file],
        ),

        # Waypoint Follower
        Node(
            package='nav2_waypoint_follower',
            executable='waypoint_follower',
            name='waypoint_follower',
            output='screen',
            parameters=[params_file],
        ),

        # Lifecycle Manager
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager',
            output='screen',
            parameters=[params_file],
        ),

        # RViz for Navigation Visualization
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', default_rviz],
            parameters=[
                {'use_sim_time': use_sim_time},
                {'qos_overrides./scan.depth': 10},
                {'qos_overrides./map.depth': 1},
                {'qos_overrides./tf.depth': 100},
            ],
            output='screen',
            additional_env={'RCUTILS_LOGGING_BUFFERED_STREAM': '1'}
        ),
    ])
