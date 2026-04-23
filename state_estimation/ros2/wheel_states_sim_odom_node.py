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

        self.R = 0.18      # wheel radius
        self.L = 1.6       # wheelbase distance
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

        self.last_time = None

    def wheel_callback(self, msg):
        now = self.get_clock().now()

        if self.last_time is None:
            self.last_time = now
            return

        dt = (now - self.last_time).nanoseconds * 1e-9
        self.last_time = now

        rpm_mean = (msg.fl_rpm + msg.fr_rpm + msg.rl_rpm + msg.rr_rpm) / 4.0
        vx = rpm_mean * 2 * np.pi * self.R / 60.0

        delta = (msg.fl_steering_angle + msg.fr_steering_angle) / 2.0
        
        wz = vx / self.L * np.tan(delta)
        
        vy = 0.0

        self.yaw += wz * dt

        self.x += vx * np.cos(self.yaw) * dt
        self.y += vx * np.sin(self.yaw) * dt

        odom = Odometry()
        odom.header = msg.header
        odom.header.frame_id = "fsds/odom"
        odom.child_frame_id = "fsds/FSCar"

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y

        #transforming yaw to quaternion
        odom.pose.pose.orientation.z = np.sin(self.yaw / 2.0)
        odom.pose.pose.orientation.w = np.cos(self.yaw / 2.0)

        odom.twist.twist.linear.x = vx
        odom.twist.twist.linear.y = vy
        odom.twist.twist.angular.z = wz

        odom.pose.covariance = [
            1.0,  0.0,   0.0,   0.0,   0.0,   0.0,
            0.0,   1.0,  0.0,   0.0,   0.0,   0.0,
            0.0,   0.0,   1e6,   0.0,   0.0,   0.0,
            0.0,   0.0,   0.0,   1e6,   0.0,   0.0,
            0.0,   0.0,   0.0,   0.0,   1e6,   0.0,
            0.0,   0.0,   0.0,   0.0,   0.0,   0.1
        ]

        odom.twist.covariance = [
            0.02,  0.0,   0.0,   0.0,   0.0,   0.0,
            0.0,   0.1,   0.0,   0.0,   0.0,   0.0,
            0.0,   0.0,   1e6,   0.0,   0.0,   0.0,
            0.0,   0.0,   0.0,   1e6,   0.0,   0.0,
            0.0,   0.0,   0.0,   0.0,   1e6,   0.0,
            0.0,   0.0,   0.0,   0.0,   0.0,   0.5
        ]

        self.odom_pub.publish(odom)

def main(args=None):
    rclpy.init()
    wheel_rpm_odom = WheelRpmSimOdom()
    rclpy.spin(wheel_rpm_odom)
    wheel_rpm_odom.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()