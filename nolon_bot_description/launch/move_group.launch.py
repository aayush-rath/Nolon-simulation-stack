#!/usr/bin/env python3
"""
Launch MoveIt 2 for 6-DOF Robotic Arm
Author: Aayush Rath | Date: 26/08/2025

Loads the robot description, MoveIt 2 config, and starts move_group.
Assumes ros2_control is already defined in the robot URDF and controllers
are spawned by a separate launch file.
"""

import os
import yaml
import xacro

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def load_yaml(abs_path):
    if not os.path.exists(abs_path):
        raise FileNotFoundError(f"YAML file not found: {abs_path}")
    with open(abs_path, "r") as f:
        data = yaml.safe_load(f)
        if data is None:
            raise ValueError(f"YAML file is empty: {abs_path}")
        return data


def generate_launch_description():
    pkg_share = get_package_share_directory('nolon_bot_description')
    urdf_path = os.path.join(pkg_share, 'urdf', 'robotic_arm/robotic_arm.urdf.xacro')
    SRDF_XACRO = os.path.join("srdf", "robotic_arm.srdf.xacro")

    ENABLE_RVIZ = True
    ENABLE_SERVO = True
    USE_SIM_TIME = True
    LOG_LEVEL = "info"
    # A generic MoveIt RViz config; replace with your own if you have one
    RVIZ_CONFIG = os.path.join(
        pkg_share,
        "rviz",
        "nolon_bot_moveit.rviz",
    )

    urdf_doc = xacro.process_file(urdf_path)
    robot_description = {"robot_description": urdf_doc.toxml()}

    srdf_path = os.path.join(pkg_share, SRDF_XACRO)
    srdf_doc = xacro.process_file(srdf_path)

    servo_params = {
        "moveit_servo": load_yaml(
            os.path.join(pkg_share, "moveit_config", "servo.yaml")
        )
    }
    servo_params["moveit_servo"].update({"use_gazebo": USE_SIM_TIME})

    robot_description_semantic = {
        "robot_description_semantic": srdf_doc.toxml()
    }

    robot_description_kinematics = {
        "robot_description_kinematics": load_yaml(
            os.path.join(pkg_share, "moveit_config", "kinematics.yaml")
        )
    }

    joint_limits = {
        "robot_description_planning": load_yaml(
            os.path.join(pkg_share, "moveit_config", "joint_limits.yaml")
        )
    }

    planning_pipeline = {
        'default_planning_pipeline': 'ompl',
        'planning_pipelines': ['ompl'],
    }

    planning_pipeline['ompl'] = {
        'planning_plugins': ['ompl_interface/OMPLPlanner'],
        'request_adapters': [
            'default_planning_request_adapters/ResolveConstraintFrames',
            'default_planning_request_adapters/ValidateWorkspaceBounds',
            'default_planning_request_adapters/CheckStartStateBounds',
            'default_planning_request_adapters/CheckStartStateCollision',
        ],
        'response_adapters': [
            'default_planning_response_adapters/AddTimeOptimalParameterization',
            'default_planning_response_adapters/ValidateSolution',
            'default_planning_response_adapters/DisplayMotionPath',
        ],
    }

    planning_pipeline['ompl'].update(load_yaml(os.path.join(pkg_share, "moveit_config", "ompl_planning.yaml")))

    planning_scene_monitor_parameters = {
        "publish_planning_scene": True,
        "publish_geometry_updates": True,
        "publish_state_updates": True,
        "publish_transforms_updates": True,
    }

    moveit_controller_manager = {
        "moveit_controller_manager": "moveit_simple_controller_manager/MoveItSimpleControllerManager",
        "moveit_simple_controller_manager": load_yaml(
            os.path.join(pkg_share, "moveit_config", "moveit_controllers.yaml")
        ),
    }

    trajectory_execution = {
        "allow_trajectory_execution": True,
        "moveit_manage_controllers": False,
        "execution_duration_monitoring": False,
        "trajectory_execution.allowed_execution_duration_scaling": 1.5,
        "trajectory_execution.allowed_goal_duration_margin": 0.5,
        "trajectory_execution.allowed_start_tolerance": 0.01,
    }

    # -----------------------------
    # Nodes
    # -----------------------------
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        arguments=["--ros-args", "--log-level", LOG_LEVEL],
        parameters=[robot_description, {"use_sim_time": USE_SIM_TIME}],
    )

    moveit_servo = Node(
        package="moveit_servo",
        executable="servo_node",
        output="log",
        arguments=["--ros-args", "--log-level", LOG_LEVEL],
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            joint_limits,
            planning_pipeline,
            trajectory_execution,
            planning_scene_monitor_parameters,
            servo_params,
            {"use_sim_time": USE_SIM_TIME}
        ]
    )

    move_group = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        arguments=["--ros-args", "--log-level", LOG_LEVEL],
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            joint_limits,
            planning_pipeline,
            trajectory_execution,
            planning_scene_monitor_parameters,
            moveit_controller_manager,
            {"use_sim_time": USE_SIM_TIME},
        ],
    )

    nodes = [robot_state_publisher, moveit_servo, move_group]

    if ENABLE_RVIZ:
        rviz2 = Node(
            package="rviz2",
            executable="rviz2",
            output="screen",
            arguments=["-d", RVIZ_CONFIG],
            parameters=[
                robot_description,
                robot_description_semantic,
                robot_description_kinematics,
                joint_limits,
                planning_pipeline,
                {"use_sim_time": USE_SIM_TIME},
            ],
        )
        nodes.append(rviz2)

    return LaunchDescription(nodes)
