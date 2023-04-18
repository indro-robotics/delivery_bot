#!/usr/bin/env python3

import os
import sys

from ament_index_python.packages import get_package_share_directory
from ament_index_python.packages import get_package_prefix
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
import xacro


def generate_launch_description():

    # Getting Installation Directorys
    pkg_deliverybot_description = get_package_share_directory(
        'deliverybot_description')
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

    # Acquiring robot description XACRO file
    xacro_file = os.path.join(
        pkg_deliverybot_description, 'models/deliverybot/xacro', 'deliverybot.xacro')
    assert os.path.exists(
        xacro_file), "The deliverybot.xacro doesn't exist in " + str(xacro_file)

    robot_description_config = xacro.process_file(xacro_file)
    robot_description = robot_description_config.toxml()
    robot_description_param = {'robot_description' : robot_description}
    #robot_description = {'robot_description': robot_description_xml}

    # FROM ONLINE TUTORIAL TEST
    controllers_file = os.path.join(
        pkg_deliverybot_description, 'models/deliverybot/config', 'controllers.yaml')

    # Arguments
    ld = LaunchDescription()

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        #parameters=[robot_description],
        parameters=[robot_description_param],
    )

    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(get_package_share_directory(
            'deliverybot_description'), 'rviz', 'eva_simulation.rviz')]
    )
    spawn_deliverybot_node = Node(
        package='deliverybot_description',
        executable='spawn_deliverybot',
        arguments=[robot_description],
        output='screen',
    )

    sim_time_argument = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true',
    )

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
    ld.add_action(sim_time_argument)
    # Robot States
    ld.add_action(robot_state_publisher_node)

    # ld.add_action(joints_state_publisher_node)

    # Robot Visualization
    ld.add_action(rviz2_node)
    ld.add_action(spawn_deliverybot_node)
    # Load Controllers

    ld.add_action(joint_state_broadcaster)
    ld.add_action(forward_position_controller)
    ld.add_action(forward_velocity_controller)
    ld.add_action(door_position_controller)

    return ld
