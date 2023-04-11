from launch import LaunchDescription

import launch.actions
import launch_ros.actions


def generate_launch_description():

    return LaunchDescription([
        launch_ros.actions.Node(
            package='deliverybot_gazebo',
            executable='wheel_steer',
            output='screen',
            arguments=["0.5"]),
    ])