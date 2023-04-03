#!/usr/bin/python3
# -*- coding: utf-8 -*-
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    pkg_deliverybot_gazebo = get_package_share_directory('deliverybot_gazebo')
    pkg_deliverybot_description = get_package_share_directory('deliverybot_description')

    ld = LaunchDescription()

    # Sart World
    start_world = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_deliverybot_gazebo, 'launch', 'world_start.launch.py'),
        )
    )

    spawn_robot_world = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_deliverybot_description, 'launch', 'deliverybot_spawn.launch.py'),
        )
    )     

    ld.add_action(start_world)
    ld.add_action(spawn_robot_world)
    return ld