#!/usr/bin/env python3

import rclpy
import rclpy.time
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from std_msgs.msg import Header
from geometry_msgs.msg import TwistWithCovariance
from geometry_msgs.msg import PoseWithCovariance
from message_filters import Subscriber
from geometry_msgs.msg import Pose


class PosePublisher(Node):

    def __init__(self):
        super().__init__('pose_publisher')
        self.publisher_pose = self.create_publisher(Pose, 'pose', 10)
        self.subscription_odometry = self.create_subscription(Odometry, 'odometry',self.message_callback, 10)
        self.i = 0

    def message_callback(self,msg):
                 
        self.publisher_pose.publish(msg.pose.pose)
        self.get_logger().info("Publishing Pose info:"f"{msg.pose.pose}) {self.i}")
        self.i += 1

def main(args=None):
    rclpy.init(args=args)

    publisher = PosePublisher()

    rclpy.spin(publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()