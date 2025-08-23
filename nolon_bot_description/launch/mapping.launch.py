"""
Launch file for SLAM Toolbox with RViz visualization - IMPROVED VERSION
Author: Aayush Rath | Date: 16/08/2025

This launch file starts:
- SLAM Toolbox for mapping with optimized settings
- RViz with custom configuration and improved message handling
- Added parameters to prevent message queue overflow
"""

import os

from launch import LaunchDescription
from ament_index_python import get_package_share_directory
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node

def generate_launch_description():
    pkg_slam_toolbox = get_package_share_directory('slam_toolbox')
    slam_launch_path = PathJoinSubstitution([pkg_slam_toolbox, 'launch', 'online_async_launch.py'])

    pkg_share = get_package_share_directory('nolon_bot_description')
    slam_config_path = os.path.join(pkg_share, 'config', 'nolon_bot_mapper.yaml')
    rviz_config_path = os.path.join(pkg_share, 'rviz', 'nolon_bot_mapper.rviz')

    # Declare launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time if true'
    )

    # SLAM Toolbox Node with improved parameters
    slam_toolbox_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(slam_launch_path),
        launch_arguments={
            "slam_params_file": slam_config_path,
            "use_sim_time": "true"
        }.items()
    )

    # RViz Node with improved message handling
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_path],
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
            {'qos_overrides./scan.depth': 10},
            {'qos_overrides./map.depth': 1},
            {'qos_overrides./tf.depth': 100},
        ],
        output='screen',
        additional_env={'RCUTILS_LOGGING_BUFFERED_STREAM': '1'}
    )

    return LaunchDescription([
        use_sim_time_arg,
        slam_toolbox_node,
        rviz_node
    ])