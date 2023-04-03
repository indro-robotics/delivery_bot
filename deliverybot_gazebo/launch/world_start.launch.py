#!/usr/bin/python3
# -*- coding: utf-8 -*-
import os
import sys

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():

    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_box_car_gazebo = get_package_share_directory('deliverybot_gazebo')


    ld = LaunchDescription()
    # Gazebo launch
    



    gazebo_launch_node = ExecuteProcess(
        cmd=['gazebo', '--verbose', 'worlds/empty_world.world', '-s','libgazebo_ros_factory.so', '-s', 'libgazebo_ros_init.so'],
        output='screen',
    )
    world_launch_argument = DeclareLaunchArgument(
        'world',
        default_value=[os.path.join(
            pkg_box_car_gazebo, 'worlds', 'empty_world.world'), ''],
        description='SDF world file')
    
    ld.add_action(gazebo_launch_node)
    ld.add_action(world_launch_argument)

    return ld
