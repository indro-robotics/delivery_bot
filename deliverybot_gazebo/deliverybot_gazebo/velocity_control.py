#! /usr/bin/env python

import sys
import rclpy
from rclpy.duration import Duration
from rclpy.action import ActionClient
from rclpy.node import Node
from control_msgs.action import FollowJointTrajectory
from control_msgs.msg import JointTrajectoryControllerState
from trajectory_msgs.msg import JointTrajectoryPoint, JointTrajectory
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
import math
from math import isclose
# ros2 action list -t
# ros2 action info /joint_trajectory_controller/follow_joint_trajectory -t
# ros2 interface show control_msgs/action/FollowJointTrajectory


class VelocityControlNode(Node):

    def __init__(self):
        super().__init__('velocity_control')

        # Initial Publisher Parameters
        self.even = 1
        self.check_count = 0
        self.cmdVel_prev = 0

        # Declaring Robot Description Parameters
        self.base_length = 0.85
        self.base_width = 0.30
        self.joint_names = ["rear_right_wheel_joint",
                            "rear_left_wheel_joint",]

        # Declaring Initial Angle Parameters
        self.vel_array = [0.0, 0.0]
        self.vel_cmd = 0

        # Creating Nodes
        self.velocity_publisher_ = self.create_publisher(JointTrajectoryControllerState, '/rear_wheel_controller/joint_trajectory_controller_states', 10)
        timer_period = 0.1
        self.velocity_timer_ = self.create_timer(timer_period, self.velocity_timer_callback)
        self._cmdVel_subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmdVel_callback,
            10)

        self._jointStates_subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.jointStates_callback,
            10
        )
    

    def jointStates_callback(self, msg):
        joint_positions = msg.position
        self.positions_prev = [joint_positions[0], joint_positions[1]]

    def cmdVel_callback(self, msg):
        self.vel_cmd_prev = self.vel_cmd
        self.vel_cmd = msg.linear.x
        self.vel_array = [self.vel_cmd, self.vel_cmd]
    
    def velocity_timer_callback(self):
        msg = JointTrajectoryControllerState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.joint_names = self.joint_names

        point = JointTrajectoryPoint()
        #point.velocities = self.vel_array
        point.velocities = [0.5, 0.5]
        point.time_from_start = Duration(seconds=0.1, nanoseconds=0).to_msg()
        msg.desired = point
        self.velocity_publisher_.publish(msg)

def main(args=None):

    rclpy.init()

    velocity_control = VelocityControlNode()

    rclpy.spin(velocity_control)
    velocity_control.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
