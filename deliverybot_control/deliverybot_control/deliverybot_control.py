#!/usr/bin/python3
import math
import threading
import rclpy
from rclpy.duration import Duration
from rclpy.action import ActionClient
import numpy as np
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from sensor_msgs.msg import JointState


vel_msg = Twist()


class DeliverybotControlNode(Node):
    def __init__(self):
        super().__init__('deliverybot_control')
        timer_period = 0.02
        self.base_length = 0.75
        self.base_width = 0.30
        self.wheel_radius = 0.0125
        self.steer_offset = 0.15  # CHECK VALUE
        self.steering_track = self.base_width - 2*self.steer_offset
        self.vel_msg_prev = 0.0

        self.pos = np.array([0, 0], float)
        self.vel = np.array([0, 0], float)

        self.pub_pos_ = self.create_publisher(
            Float64MultiArray, '/forward_position_controller/commands', 10)
        self.pub_vel_ = self.create_publisher(
            Float64MultiArray, '/forward_velocity_controller/commands', 10)
        self.timer_ = self.create_timer(timer_period, self.timer_callback)

        self._jointStates_subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.jointStates_callback,
            10
        )

        self._action_client = ActionClient(
            self, FollowJointTrajectory, '/door_position_controller/follow_joint_trajectory')
        

    def theta_out(self, delta_ack):
        return math.atan((self.base_length*math.atan(delta_ack))/(self.base_length + 0.5*self.base_width*math.atan(delta_ack)))

    def theta_in(self, delta_ack):
        return math.atan((self.base_length*math.atan(delta_ack))/(self.base_length - 0.5*self.base_width*math.atan(delta_ack)))

    def timer_callback(self):
        global vel_msg

        sign = np.sign(vel_msg.linear.x)
        if vel_msg.angular.z > 0:
            self.pos[0] = self.theta_in(vel_msg.angular.z)
            self.vel[0] = sign * np.abs(vel_msg.linear.x * \
                (np.sin(vel_msg.angular.z) / np.sin(self.pos[0])))
            self.pos[1] = self.theta_out(vel_msg.angular.z)
            self.vel[1] = sign * np.abs(vel_msg.linear.x * \
                (np.sin(vel_msg.angular.z) / np.sin(self.pos[1])))
        if vel_msg.angular.z < 0:
            self.pos[0] = self.theta_out(vel_msg.angular.z)
            self.vel[0] = sign * np.abs(vel_msg.linear.x * \
                (np.sin(vel_msg.angular.z) / np.sin(self.pos[0])))
            self.pos[1] = self.theta_in(vel_msg.angular.z)
            self.vel[1] = sign * np.abs(vel_msg.linear.x * \
                (np.sin(vel_msg.angular.z) / np.sin(self.pos[1])))
        if vel_msg.angular.z == 0:
            self.pos[0] = 0
            self.vel[0] = vel_msg.linear.x
            self.pos[1] = 0
            self.vel[1] = vel_msg.linear.x

        pos_array = Float64MultiArray(data=self.pos)
        vel_array = Float64MultiArray(data=self.vel)

        self.pub_pos_.publish(pos_array)
        self.pub_vel_.publish(vel_array)
        self.pos[:] = 0
        self.vel[:] = 0

    def jointStates_callback(self, msg):
        global vel_msg
        joint_positions = msg.position
        self.positions_prev = [joint_positions[2]]
        if vel_msg.linear.z != self.vel_msg_prev:
            self.get_logger().info('Door Toggled.')
            self.send_door_goal()

    def send_door_goal(self):
        global vel_msg
        self.vel_msg_prev = vel_msg.linear.z
        goal_msg = FollowJointTrajectory.Goal()
        points = []
        point1 = JointTrajectoryPoint()
        point1.positions = self.positions_prev

        point2 = JointTrajectoryPoint()
        point2.time_from_start = Duration(seconds=3, nanoseconds=0).to_msg()

        if vel_msg.linear.z > 0:
            self.positions = [-1.5708]

        if vel_msg.linear.z < 0:
            self.positions = [0.0]
        if vel_msg.linear.z == 0:
            self.positions = self.positions_prev

        if point1 == point2:
            return
        point2.positions = self.positions
        points = [point1, point2]

        goal_msg.goal_time_tolerance = Duration(
            seconds=1, nanoseconds=0).to_msg()
        goal_msg.trajectory.joint_names = ['door_joint']
        goal_msg.trajectory.points = points

        self._action_client.wait_for_server()
        self._send_door_goal_future = self._action_client.send_goal_async(
            goal_msg, feedback_callback=self.feedback_callback)

        self._send_door_goal_future.add_done_callback(self.goal_response_callback)
        
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
        #self.get_logger().info('Result: '+str(result))

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback

class JoySubscriberNode(Node):

    def __init__(self):
        super().__init__('joy_subscriber')
        self.subscription_ = self.create_subscription(
            Joy,
            'joy',
            self.listener_callback,
            10
        )
        self.subscription_

    def listener_callback(self, data):
        global vel_msg

        vel_msg.linear.x = data.axes[1]*7.5
        vel_msg.linear.y = data.axes[0]*7.5
        vel_msg.angular.z = data.axes[3]*10


class CmdVelSubscriberNode(Node):

    def __init__(self):
        super().__init__('cmd_vel_subscriber')
        self.cmdVel_subscription_ = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmdVel_callback,
            10)
    def cmdVel_callback(self, data):
        global vel_msg
        vel_msg = data
        #super().send_door_goal()




def main(args=None):
    rclpy.init(args=None)

    deliverybotControl = DeliverybotControlNode()
    cmdVelSubscriber = CmdVelSubscriberNode()

    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(deliverybotControl)
    executor.add_node(cmdVelSubscriber)

    executor_thread = threading.Thread(target=executor.spin, daemon=True)
    executor_thread.start()

    rate=deliverybotControl.create_rate(2)
    try:
        while rclpy.ok():
            rate.sleep()
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()
    executor_thread.join()

if __name__ == '__main__':
    main()