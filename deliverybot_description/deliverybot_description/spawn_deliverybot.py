#!/usr/bin/env python3
import os
import sys
import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import SpawnEntity
from functools import partial


from ament_index_python.packages import get_package_share_directory
from ament_index_python.packages import get_package_prefix
import xacro


class SpawnDeliverybotNode(Node):
    def __init__(self):
        super().__init__("minimal_client")
        client = self.create_client(SpawnEntity, '/spawn_entity')
        # content = sys.argv[1]

        pkg_deliverybot_description = get_package_share_directory(
            'deliverybot_description')
        install_dir = get_package_prefix('deliverybot_description')

        xacro_file = os.path.join(
            pkg_deliverybot_description, 'robot', 'deliverybot.xacro')
        assert os.path.exists(
            xacro_file), "The deliverybot.xacro doesn't exist in " + str(xacro_file)

        robot_description_config = xacro.process_file(xacro_file)
        robot_description = robot_description_config.toxml()

        req = SpawnEntity.Request()
        req.name = "deliverybot"
        req.xml = robot_description
        req.robot_namespace = ""
        req.reference_frame = "world"
        req.initial_pose.position.x = 0.0
        req.initial_pose.position.y = 0.0
        req.initial_pose.position.z = 0.2

        while not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('Service not available, waiting again...')

        future = client.call_async(req)
        future.add_done_callback(partial(self.callback_spawn_entity))

    def callback_spawn_entity(self, future):
        try:
            response = future.result()
        except Exception as e:
            self.get_logger().error("Service call failed %r" % (e,))


def main(args=None):
    rclpy.init(args=args)
    node = SpawnDeliverybotNode()
    rclpy.spin(node)
    rclpy.shutdown()
