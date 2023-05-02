#!/usr/bin/env python3

import os
import sys

from ament_index_python.packages import get_package_share_directory
from ament_index_python.packages import get_package_prefix
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
import xacro

from launch.actions import (
    DeclareLaunchArgument, 
    IncludeLaunchDescription,
    ExecuteProcess
)



def generate_launch_description():

    # Getting Installation Directorys
    pkg_deliverybot_description = get_package_share_directory(
        'deliverybot_description')
    pkg_deliverybot_control = get_package_share_directory('deliverybot_control')

    ld = LaunchDescription()

    # Acquiring robot description XACRO file
    xacro_file = os.path.join(
        pkg_deliverybot_description, 'models/deliverybot/xacro', 'nav2.xacro')
    assert os.path.exists(
        xacro_file), "The deliverybot.xacro doesn't exist in " + str(xacro_file)

    robot_description_config = xacro.process_file(xacro_file)
    robot_description = robot_description_config.toxml()
    robot_description_param = {'robot_description': robot_description}

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[robot_description_param],
    )

    spawn_deliverybot_node = Node(
        package='deliverybot_description',
        executable='spawn_deliverybot',
        arguments=[robot_description],
        output='screen',
    )

    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(get_package_share_directory(
            'deliverybot_control'), 'rviz', 'teleop.rviz')]
    )

    robot_localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [pkg_deliverybot_control, '/launch/robot_localization.launch.py']
        )
    )

    ld.add_action(robot_state_publisher_node)
    ld.add_action(spawn_deliverybot_node)
    ld.add_action(rviz2_node)
    ld.add_action(robot_localization_launch)

    return ld
