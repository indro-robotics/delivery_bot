#!/usr/bin/python3
# -*- coding: utf-8 -*-
import os
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, TextSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration


def generate_launch_description():

    pkg_deliverybot_description = get_package_share_directory('deliverybot_description')
    pkg_deliverybot_gazebo = get_package_share_directory('deliverybot_gazebo')
    gazebo_world = os.path.join(
        pkg_deliverybot_gazebo, 'worlds','obstacle_simulation.sdf')

    ld = LaunchDescription()

    sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='true')
            # Declaring Arguments and Configurations
    teleop_only = LaunchConfiguration('teleop_only')

    teleop_arg = DeclareLaunchArgument(
        'teleop_only', default_value='false', description='Launching simulation in teleop only mode')
    
    spawn_robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_deliverybot_description, 'launch', 'deliverybot_spawn.launch.py'),
        )
    )

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
    robot_control_node = Node(
        package='deliverybot_control',
        executable='deliverybot_control',
        output='screen',
    )

    # Launching Simulation Type
    slam_simulation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [pkg_deliverybot_gazebo, '/launch/nav2.launch.py']),
        condition=UnlessCondition(teleop_only)
    )
    teleop_simulation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [pkg_deliverybot_gazebo, '/launch/teleop.launch.py']),
        condition=IfCondition(teleop_only)
    )
    #Adding arguments
    ld.add_action(sim_time)
    ld.add_action(teleop_arg)
    # Launching Gazebo Server
    ld.add_action(gzserver_launch)
    # Launching Gazebo Client
    ld.add_action(gzclient_launch)
    # Spawning the robot
    ld.add_action(spawn_robot)
    #Launching control node
    ld.add_action(robot_control_node)
    #Simulation
    ld.add_action(slam_simulation_launch)
    ld.add_action(teleop_simulation_launch)

    return ld
