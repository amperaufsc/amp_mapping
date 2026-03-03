#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from message_filters import Subscriber, ApproximateTimeSynchronizer
from nav_msgs.msg import Odometry


class EkfOdomSimNode(Node):
    def __init__(self):
        super().__init__('ekf_odom_sim_node')

        self.ekf_odom_sub = Subscriber(self, Odometry, 'Ekf_odom_sub')
        self.fsds_odom_sub = Subscriber(self, Odometry, 'fsds_odom_sub')

        self.odom_pub = self.create_publisher(Odometry, 'odom_pub', 10)
        
        queue_size = 10
        max_delay = 0.1
        
        self.time_sync = ApproximateTimeSynchronizer([self.ekf_odom_sub, self.fsds_odom_sub], queue_size, max_delay)
        self.time_sync.registerCallback(self.odom_callback)
    

    def odom_callback(self, fsds_odom_msg, ekf_odom_msg):
        odom = Odometry()
        odom.header = fsds_odom_msg.header

        odom.pose.pose.position.x = ekf_odom_msg.pose.pose.position.x
        odom.pose.pose.position.y = ekf_odom_msg.pose.pose.position.y
        odom.pose.pose.position.z = ekf_odom_msg.pose.pose.position.z

        odom.pose.pose.orientation.x = ekf_odom_msg.pose.pose.orientation.x
        odom.pose.pose.orientation.y = ekf_odom_msg.pose.pose.orientation.y
        odom.pose.pose.orientation.z = ekf_odom_msg.pose.pose.orientation.z
        odom.pose.pose.orientation.w = fsds_odom_msg.pose.pose.orientation.w

        odom.twist.twist.linear.x = ekf_odom_msg.twist.twist.linear.x
        odom.twist.twist.linear.y = ekf_odom_msg.twist.twist.linear.y
        odom.twist.twist.linear.z = ekf_odom_msg.twist.twist.linear.z

        odom.twist.twist.angular.x = fsds_odom_msg.twist.twist.angular.x
        odom.twist.twist.angular.y = fsds_odom_msg.twist.twist.angular.y
        odom.twist.twist.angular.z = fsds_odom_msg.twist.twist.angular.z

        self.odom_pub.publish(odom)


def main(args=None):
    rclpy.init()
    ekf_odom_sim_node = EkfOdomSimNode()
    rclpy.spin(ekf_odom_sim_node)
    ekf_odom_sim_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()