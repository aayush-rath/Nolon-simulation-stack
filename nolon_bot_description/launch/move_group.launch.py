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


# -----------------------------
# Simple toggles / paths
# -----------------------------
PKG = "nolon_bot_description"
URDF_XACRO = os.path.join("urdf", "nolon_bot.urdf.xacro")
SRDF_XACRO = os.path.join("srdf", "robotic_arm.srdf.xacro")
MOVEIT_CFG_DIR = "moveit_config"

ENABLE_RVIZ = True
USE_SIM_TIME = True
LOG_LEVEL = "info"
# A generic MoveIt RViz config; replace with your own if you have one
RVIZ_CONFIG = os.path.join(
    PKG,
    "rviz",
    "moveit.rviz",
)


def load_yaml(pkg: str, rel_path: str):
    base = get_package_share_directory(pkg)
    abs_path = os.path.join(base, rel_path)
    with open(abs_path, "r") as f:
        return yaml.safe_load(f)


def generate_launch_description():
    pkg_share = get_package_share_directory(PKG)

    # -----------------------------
    # Build robot_description (URDF)
    # -----------------------------
    urdf_path = os.path.join(pkg_share, URDF_XACRO)
    urdf_doc = xacro.process_file(urdf_path)
    robot_description = {"robot_description": urdf_doc.toxml()}

    # -----------------------------
    # Build robot_description_semantic (SRDF)
    # -----------------------------
    srdf_path = os.path.join(pkg_share, SRDF_XACRO)
    srdf_doc = xacro.process_file(srdf_path)
    robot_description_semantic = {
        "robot_description_semantic": srdf_doc.toxml()
    }

    # -----------------------------
    # MoveIt configs
    # -----------------------------
    robot_description_kinematics = {
        "robot_description_kinematics": load_yaml(
            PKG, os.path.join(MOVEIT_CFG_DIR, "kinematics.yaml")
        )
    }

    joint_limits = {
        "robot_description_planning": load_yaml(
            PKG, os.path.join(MOVEIT_CFG_DIR, "joint_limits.yaml")
        )
    }

    planning_pipeline = {
        "planning_pipelines": ["ompl"],
        "default_planning_pipeline": "ompl",
        "ompl": load_yaml(PKG, os.path.join(MOVEIT_CFG_DIR, "ompl_planning.yaml")),
    }

    planning_scene_monitor_parameters = {
        "publish_planning_scene": True,
        "publish_geometry_updates": True,
        "publish_state_updates": True,
        "publish_transforms_updates": True,
    }

    moveit_controller_manager = {
        "moveit_controller_manager": "moveit_simple_controller_manager/MoveItSimpleControllerManager",
        "moveit_simple_controller_manager": load_yaml(
            PKG, os.path.join(MOVEIT_CFG_DIR, "moveit_controllers.yaml")
        ),
    }

    trajectory_execution = {
        "allow_trajectory_execution": True,
        # Controllers are spawned/managed elsewhere (e.g., controller_manager spawners)
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

    nodes = [robot_state_publisher, move_group]

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
