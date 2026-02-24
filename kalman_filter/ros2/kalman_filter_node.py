#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry

class KalmanFilterNode(Node):
    def __init__(self):
        super().__init__('kalman_filter_node')
        self.subscription = self.create_subscription(Odometry, 'odom', self.odom_callback, 10)



def main(args=None):
    rclpy.init(args=args)
    kalman_filter_node = KalmanFilterNode()
    rclpy.spin(kalman_filter_node)
    kalman_filter_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()