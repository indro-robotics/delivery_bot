#!/usr/bin/env python3

import os
import xacro

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration


def generate_launch_description():

    deliverybot_control_pkg = get_package_share_directory('deliverybot_control')
    ekf_config = os.path.join(deliverybot_control_pkg, 'config/ekf.yaml')
    ld = LaunchDescription()

    sim_time_argument = DeclareLaunchArgument(name='use_sim_time', default_value='True',
                                                    description='Flag to enable use_sim_time')

    robot_localization_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_config, {'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )

    ld.add_action(sim_time_argument)
    ld.add_action(robot_localization_node)

    return ld