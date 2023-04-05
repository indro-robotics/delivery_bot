#!/usr/bin/python3
# -*- coding: utf-8 -*-
import os
import sys

from ament_index_python.packages import get_package_share_directory
from ament_index_python.packages import get_package_prefix

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, TextSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_deliverybot_gazebo = get_package_share_directory('deliverybot_gazebo')
    gazebo_world = os.path.join(
        pkg_deliverybot_gazebo, 'worlds','obstacle_simulation.sdf')

    ld = LaunchDescription()

    gzserver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('gazebo_ros'),
                'launch', 'gzserver.launch.py',
            ])
        ]),
        launch_arguments={
            'verbose': 'true',
            'world': TextSubstitution(text=str(gazebo_world))
        }.items()
    )

    gzclient_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('gazebo_ros'),
                'launch', 'gzclient.launch.py'
            ])
        ]),
        launch_arguments={
            'verbose': 'false',
        }.items()
    )

    # Launching Gazebo Server
    ld.add_action(gzserver_launch)
    # Launching Gazebo Client
    ld.add_action(gzclient_launch)

    return ld