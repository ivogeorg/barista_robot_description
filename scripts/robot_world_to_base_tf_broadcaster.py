#! /usr/bin/env python3

import sys
import rclpy
from rclpy.node import Node
from rclpy.qos import ReliabilityPolicy, DurabilityPolicy, QoSProfile
import tf2_ros
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry

# Connect robot base (child) to world (parent) using Odometry
# Publish a transform from world (parent) ro Odometry of base link
class RobotWorldToBaseTfBroadcaster(Node):

    def __init__(self, robot_name="generic_robot", robot_base_frame="base_footprint"):
        super().__init__(robot_name + '_world_to_base_broadcaster_node')

        self.robot_name = robot_name
        self.robot_base_frame = robot_base_frame
        self.init_tf_message()

        self.subscriber = self.create_subscription(
            Odometry,
            '/' + self.robot_name + '/odom',
            self.odom_callback,
            QoSProfile(depth=1, durability=DurabilityPolicy.VOLATILE, reliability=ReliabilityPolicy.BEST_EFFORT))

        self.broadcaster = tf2_ros.TransformBroadcaster(self)

        # self.get_logger().warning("odom_to_tf_broadcaster_node READY!")
        self.get_logger().info(self.robot_name + "_world_to_base_broadcaster_node READY!")

    def odom_callback(self, msg):
        # record latest odometry
        self.update_data(msg)

        # print the log info in the terminal
        self.get_logger().debug('/' + self.robot_name + '/odom/value: "%s"' % str(self.robot_odom))

        # broadcast transform from world to robot base
        self.broadcast_new_tf()

    def update_data(self, msg):
        self.robot_odom = msg

    def init_tf_message(self):
        self.transform_stamped = TransformStamped()
        self.transform_stamped.header.frame_id = "world"
        self.transform_stamped.child_frame_id = self.robot_base_frame

    def broadcast_new_tf(self):

        # take the time from the odom message instead of generating it from now()
        # this way it won't be "late" and discarded

        time_header = self.robot_odom.header
        position = self.robot_odom.pose.pose.position
        orientation = self.robot_odom.pose.pose.orientation

        self.transform_stamped.header.stamp = time_header.stamp
        self.transform_stamped.transform.translation.x = position.x
        self.transform_stamped.transform.translation.y = position.y
        self.transform_stamped.transform.translation.z = position.z
        self.transform_stamped.transform.rotation.x = orientation.x
        self.transform_stamped.transform.rotation.y = orientation.y
        self.transform_stamped.transform.rotation.z = orientation.z
        self.transform_stamped.transform.rotation.w = orientation.w

        self.broadcaster.sendTransform(self.transform_stamped)


def main(args=None):

    rclpy.init()

    world_to_base_tf_broadcaster = RobotWorldToBaseTfBroadcaster()

    rclpy.spin(world_to_base_tf_broadcaster)


if __name__ == '__main__':
    main()