#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import QuaternionStamped
import numpy as np

class WheelRpmOdom(Node):
    def __init__(self):
        super().__init__('wheel_rpm_odom')

        self.wheel_subscription = self.create_subscription(QuaternionStamped, '/wheel_rpm', self.wheel_callback, 10)

        self.odom_pub = self.create_publisher(Odometry, '/wheel_odom', 10)

    def wheel_callback(self, msg):
        odom = Odometry()
        rear_left_wheel = msg.quaternion.z
        rear_right_wheel = msg.quaternion.w

        mean_rpm = (rear_left_wheel + rear_right_wheel) / 2
        vx = mean_rpm * 2 * np.pi * 0.231 / (15 * 60)

        covariance_matrix = [1e-4,  0.0,   0.0,    0.0,    0.0,    0.0,
                            0.0,    1e-4,  0.0,    0.0,    0.0,    0.0,
                            0.0,    0.0,   1e-4,    0.0,    0.0,    0.0,
                            0.0,    0.0,   0.0,    1e-4,    0.0,    0.0,
                            0.0,    0.0,   0.0,    0.0,    1e-4,    0.0,
                            0.0,    0.0,   0.0,    0.0,    0.0,    1e-4]

        odom.header = msg.header
        odom.twist.twist.linear.x = vx
        odom.twist.covariance = covariance_matrix

        self.odom_pub.publish(odom)

def main(args=None):
    rclpy.init()
    wheel_rpm_odom = WheelRpmOdom()
    rclpy.spin(wheel_rpm_odom)
    wheel_rpm_odom.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()