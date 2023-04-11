#! /usr/bin/env python

import sys
import rclpy
from rclpy.duration import Duration
from rclpy.action import ActionClient
from rclpy.node import Node
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
import math
from math import isclose
# ros2 action list -t
# ros2 action info /joint_trajectory_controller/follow_joint_trajectory -t
# ros2 interface show control_msgs/action/FollowJointTrajectory

class SteeringActionClient(Node):

    def __init__(self):
        super().__init__('ackermann_steering_controller')

        #Initial Publisher Parameters
        self.even = 1
        self.check_count = 0
        self.cmdVel_prev = 0

        #Declaring Robot Description Parameters
        self.base_length = 0.85
        self.base_width = 0.30
        self.joint_names = ["front_right_steer_joint",
                        "front_left_steer_joint",]

        #Declaring Initial Angle Parameters
        self.positions_prev = [0.0, 0.0]
        self.positions = [0.0, 0.0]
        self.theta_ack = 0
        self.theta_left = 0
        self.theta_right = 0

        #Creating Nodes
        self._action_client = ActionClient(self, FollowJointTrajectory, '/ackermann_steering_controller/follow_joint_trajectory')

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
        # if joint_positions[0] == 0:
        #     self.check_count +=1
        #     if self.check_count > 3:
        #         self.positions_prev = [joint_positions[0], joint_positions[1]]
        #         self.check_count = 0


        # # print("JointPositions: " + str(joint_positions[0]))
        # # print("Previous Positions: " + str(self.positions_prev[0]))

        # #if isclose(self.positions_prev[0], joint_positions[0], abs_tol=1e-4) == True:
        # #    return
        # #else:
        self.positions_prev = [joint_positions[0], joint_positions[1]]

    def cmdVel_callback(self, msg):
        self.cmdVel_prev = self.theta_ack
        self.theta_ack = msg.angular.z
        
        if self.cmdVel_prev == self.theta_ack:
            return
        self.send_goal(self.theta_ack)

    def theta_out(self, theta_ack):
        return math.atan((self.base_length*math.atan(self.theta_ack))/(self.base_length + 0.5*self.base_width*math.atan(self.theta_ack)))
    def theta_in(self, theta_ack):
        return math.atan((self.base_length*math.atan(self.theta_ack))/(self.base_length - 0.5*self.base_width*math.atan(self.theta_ack)))

    def send_goal(self, angle):
        goal_msg = FollowJointTrajectory.Goal()
        joint_names = self.joint_names

        points = []
        point1 = JointTrajectoryPoint()
        point1.positions = self.positions_prev #FR , FL
        # point1.velocities = [1.0, 1.0]
        # point1.accelerations = [1.0, 1.0]

        point2 = JointTrajectoryPoint()
        point2.time_from_start = Duration(seconds=0.5, nanoseconds=0).to_msg()
        # point2.velocities = [1.0, 1.0]
        # point2.accelerations = [1.0, 1.0]
        
        if self.theta_ack >= 0:
            self.positions = [self.theta_in(self.theta_ack), self.theta_out(self.theta_ack)]

        if self.theta_ack < 0:
            self.positions = [self.theta_out(self.theta_ack), self.theta_in(self.theta_ack)]
        
        if point1 == point2:
            return
        point2.positions = self.positions
        points = [point1, point2]

        goal_msg.goal_time_tolerance = Duration(seconds=1, nanoseconds=0).to_msg()
        goal_msg.trajectory.joint_names = joint_names
        goal_msg.trajectory.points = points

        self._action_client.wait_for_server()
        self.get_logger().info('Ackermann Steering Points: ' + str(points))
        self._send_goal_future = self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)

        self._send_goal_future.add_done_callback(self.goal_response_callback)


    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)
    
    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info('Result: '+str(result))

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback

    

def main(args=None):
    
    rclpy.init()

    action_client = SteeringActionClient()

    rclpy.spin(action_client)
    action_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()