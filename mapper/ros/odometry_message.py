#!/usr/bin/env python3

import rclpy
import rclpy.time
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from std_msgs.msg import Header
from geometry_msgs.msg import TwistWithCovariance
from geometry_msgs.msg import PoseWithCovariance
from geometry_msgs.msg import PoseStamped
from message_filters import Subscriber

class OdometryPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisherz')
        self.publisher_ = self.create_publisher(Odometry, 'odom_pub', 10)
        self.subscription_ = self.create_subscription(PoseStamped, 'pose_sub',self.message_callback, 10)
        self.i = 0
        self.new_msg = Odometry()
        

    def message_callback(self,msg):
        
        self.new_msg.header = msg.header
        self.new_msg.pose.pose.position.x = msg.pose.position.x
        self.new_msg.pose.pose.position.y = msg.pose.position.y
        self.new_msg.pose.pose.position.z = msg.pose.position.z
        self.new_msg.pose.pose.orientation.w = msg.pose.orientation.w
        self.new_msg.pose.pose.orientation.z = msg.pose.orientation.z
        self.new_msg.pose.pose.orientation.y = msg.pose.orientation.y
        self.new_msg.pose.pose.orientation.x = msg.pose.orientation.x
        self.new_msg.twist.twist.linear.x = 1.5
        self.publisher_.publish(self.new_msg)
        
        

def main(args=None):
    rclpy.init(args=args)

    publisher = OdometryPublisher()

    rclpy.spin(publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()