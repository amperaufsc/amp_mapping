#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from fs_msgs.msg import WheelStates
import numpy as np

class WheelRpmSimOdom(Node):
    def __init__(self):
        super().__init__('wheel_states_odom')

        self.wheel_subscription = self.create_subscription(WheelStates, '/fsds/wheel_states', self.wheel_callback, 10)

        self.odom_pub = self.create_publisher(Odometry, '/wheel_odom', 10)

    def wheel_callback(self, msg):
        odom = Odometry()
        rear_left_wheel = msg.rl_rpm
        rear_right_wheel = msg.rr_rpm

        mean_rpm = (rear_left_wheel + rear_right_wheel) / 2
        vx = mean_rpm * 2 * np.pi * 0.18 / (60)

        covariance_matrix = [1e-2,  0.0,   0.0,    0.0,    0.0,    0.0,
                            0.0,    1e-2,  0.0,    0.0,    0.0,    0.0,
                            0.0,    0.0,   1e-2,    0.0,    0.0,    0.0,
                            0.0,    0.0,   0.0,    1e-2,    0.0,    0.0,
                            0.0,    0.0,   0.0,    0.0,    1e-2,    0.0,
                            0.0,    0.0,   0.0,    0.0,    0.0,    1e-2]

        odom.header = msg.header
        odom.twist.twist.linear.x = vx
        odom.twist.covariance = covariance_matrix

        self.odom_pub.publish(odom)

def main(args=None):
    rclpy.init()
    wheel_rpm_odom = WheelRpmSimOdom()
    rclpy.spin(wheel_rpm_odom)
    wheel_rpm_odom.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()