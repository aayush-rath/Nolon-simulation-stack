"""
Simulation Launch File for nolon_bot Platform
Author: Aayush Rath | Date: 10/08/2025

This launch file starts:
- Gazebo simulation with an empty world
- Robot State Publisher with the robot description
- Entity spawner for nolon_bot in Gazebo
- ROS-Gazebo bridge for clock, laser scan, and camera topics
- Joint state and mobile base controllers

Usage:
- Launch for integrated simulation, control, and mapping of nolon_bot
- Extend by adding controllers, sensors, or custom world files
"""

import os
import xacro

from launch import LaunchDescription
from ament_index_python import get_package_share_directory
from launch.actions import IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
def generate_launch_description():

    pkg_share = get_package_share_directory('nolon_bot_description')
    urdf_path = os.path.join(pkg_share, 'urdf', 'nolon_bot.urdf.xacro')
    world_path = os.path.join(pkg_share, 'world', 'empty_world.sdf')
    config_path = os.path.join(pkg_share, 'config', 'nolon_bot_control.yaml')

    # env_var = SetEnvironmentVariable(
    #     name='GZ_SIM_RESOURCE_PATH',
    #     value= os.path.join(pkg_share, 'models') + 
    #         os.environ.get('GZ_SIM_RESOURCE_PATH', '')
    # )

    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    gz_launch_path = PathJoinSubstitution([pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py'])

    doc = xacro.process_file(
        urdf_path,
        mappings={
            "use_gazebo": "true",
        }
    )
    robot_description_config = doc.toxml()

    declare_world_path = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gz_launch_path),
        launch_arguments={
            'gz_args': ['-r ', world_path],
            'on_exit_shutdown': 'True'
        }.items(),
    )

    publish_robot_state = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[
            {'robot_description': robot_description_config},
            {"use_sim_time": True}
        ]
    )

    gz_spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=['-topic', '/robot_description','-z', '0.5'],
        parameters=[{'use_sim_time': True}]
    )

    ros_gz_bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
            'rgbd_camera/depth_image@sensor_msgs/msg/Image@gz.msgs.Image',
            'rgbd_camera/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked',
        ],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    joint_state_broadcaster_spawner = RegisterEventHandler(
        OnProcessExit(
            target_action= gz_spawn_entity,
            on_exit= Node(
                package='controller_manager',
                executable='spawner',
                arguments=['joint_broad', '--controller-manager-timeout', '50'],
                parameters=[{"use_sim_time": True}]
            )
        )
    )

    mobile_base_controller_spawner = RegisterEventHandler(
            OnProcessExit(
            target_action=gz_spawn_entity,
            on_exit= Node(
                package='controller_manager',
                executable='spawner',
                arguments=['mobile_base_controller', '--param-file', config_path, '--controller-manager-timeout', '50'],
                parameters=[{"use_sim_time": True}]
            )
        )
    )

    robotic_arm_controller_spawner = RegisterEventHandler(
            OnProcessExit(
            target_action=gz_spawn_entity,
            on_exit=Node(
                package='controller_manager',
                executable='spawner',
                arguments=['robotic_arm_controller', '--param-file', config_path, '--controller-manager-timeout', '50'],
                parameters=[{"use_sim_time": True}]
            )
        )
    )

    return LaunchDescription([
        # env_var,
        declare_world_path,
        publish_robot_state,
        gz_spawn_entity,
        ros_gz_bridge_node,
        joint_state_broadcaster_spawner,
        mobile_base_controller_spawner,
        robotic_arm_controller_spawner
    ])