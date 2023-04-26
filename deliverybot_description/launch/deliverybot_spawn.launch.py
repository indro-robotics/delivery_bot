#!/usr/bin/env python3

import os
import sys

from ament_index_python.packages import get_package_share_directory
from ament_index_python.packages import get_package_prefix
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch_ros.actions import Node
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    # Getting Installation Directories
    pkg_deliverybot_gazebo = get_package_share_directory(
        'deliverybot_gazebo')
    install_dir = get_package_prefix('deliverybot_description')

    # Installing Gazebo Model and Plugin Paths
    if 'GAZEBO_MODEL_PATH' in os.environ:
        os.environ['GAZEBO_MODEL_PATH'] = os.environ['GAZEBO_MODEL_PATH'] + \
            ':' + install_dir + '/share'
    else:
        os.environ['GAZEBO_MODEL_PATH'] = install_dir + "/share"
    if 'GAZEBO_PLUGIN_PATH' in os.environ:
        os.environ['GAZEBO_PLUGIN_PATH'] = os.environ['GAZEBO_PLUGIN_PATH'] + \
            ':' + install_dir + '/lib'
    else:
        os.environ['GAZEBO_PLUGIN_PATH'] = install_dir + '/lib'

    # Creating Launch Description
    ld = LaunchDescription()

    # Declaring Arguments and Configurations
    teleop_only = LaunchConfiguration('teleop_only')

    teleop_arg = DeclareLaunchArgument(
        'teleop_only', default_value='false', description='Launching simulation in teleop only mode')

    # Launching Simulation Type
    slam_simulation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [pkg_deliverybot_gazebo, '/launch/slam.launch.py']),
        condition=UnlessCondition(teleop_only)
    )
    teleop_simulation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [pkg_deliverybot_gazebo, '/launch/teleop.launch.py']),
        condition=IfCondition(teleop_only)
    )

    # Launching Joystick Node
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node'
    )

    # Spawning Controllers
    joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster",
                   "--controller-manager", "/controller_manager"],
    )
    forward_position_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["steering_trajectory_controller",
                   "--controller-manager", "/controller_manager"],
    )
    forward_velocity_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["forward_velocity_controller",
                   "--controller-manager", "/controller_manager"],
    )
    door_position_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["door_position_controller",
                   "--controller-manager", "/controller_manager"],
    )

    # Arguments
    ld.add_action(teleop_arg)

    # Simulation Type
    ld.add_action(slam_simulation_launch)
    ld.add_action(teleop_simulation_launch)

    # Joystick Node
    ld.add_action(joy_node)

    # Controllers
    ld.add_action(joint_state_broadcaster)
    ld.add_action(forward_position_controller)
    ld.add_action(forward_velocity_controller)
    ld.add_action(door_position_controller)

    return ld
