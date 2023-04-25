#!/usr/bin/env python3

import os
import xacro

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    # Getting Installation Directorys
    pkg_deliverybot_description = get_package_share_directory(
        'deliverybot_description')
    pkg_rtabmap = get_package_share_directory('rtabmap_ros')

    # Creating Launch Description
    ld = LaunchDescription()

    # Declaring Launch Arguments
    rtabmap_args = DeclareLaunchArgument(
        'rtabmap_args', default_value='--delete_db_on_start')
    depth_topic = DeclareLaunchArgument(
        'depth_topic', default_value='/d430/front_depth_camera/depth/image_raw')
    rgb_topic = DeclareLaunchArgument(
        'rgb_topic', default_value='/d430/front_depth_camera/image_raw')
    camera_info_topic = DeclareLaunchArgument(
        'camera_info_topic', default_value='/d430/front_depth_camera/depth/camera_info')
    approx_sync = DeclareLaunchArgument(
        'approx_sync', default_value='true')
    rtabmap_viz = DeclareLaunchArgument(
        'rtabmapviz', default_value='false')
    rviz = DeclareLaunchArgument(
        'rviz', default_value='false')
    sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='true')

    ld.add_action(rtabmap_args)
    ld.add_action(depth_topic)
    ld.add_action(rgb_topic)
    ld.add_action(camera_info_topic)
    ld.add_action(approx_sync)
    ld.add_action(rtabmap_viz)
    ld.add_action(rviz)
    ld.add_action(sim_time)

    rtabmap_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [pkg_rtabmap, '/launch/rtabmap.launch.py']),
    )

    # Visualization
    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(get_package_share_directory(
            'deliverybot_control'), 'rviz', 'slam.rviz')]
    )
    # Visualization
    ld.add_action(rtabmap_launch)
    ld.add_action(rviz2_node)

    return ld
