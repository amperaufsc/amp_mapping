#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from fs_msgs.msg import WheelStates
from message_filters import Subscriber, ApproximateTimeSynchronizer
import numpy as np
from scipy.spatial.transform import Rotation as R

class WheelRpmSimOdom(Node):
    def __init__(self):
        super().__init__('wheel_states_odom')

        self.wheel_sub = Subscriber(self, WheelStates, '/fsds/wheel_states')
        self.imu_sub = Subscriber(self, Imu, '/fsds/imu') #for a better yaw value

        self.odom_pub = self.create_publisher(Odometry, '/wheel_odom', 10)

        queue_size = 10
        max_delay = 0.1    #delay in seconds
        self.time_sync = ApproximateTimeSynchronizer([self.wheel_sub, self.imu_sub], queue_size, max_delay)
        self.time_sync.registerCallback(self.wheel_callback)

        self.R = 0.18      # wheel radius
        self.L = 1.6       # wheelbase distance
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

        self.last_time = None

    def wheel_callback(self, wheel_msg, imu_msg):
        now = wheel_msg.header.stamp.sec + wheel_msg.header.stamp.nanosec * 1e-9

        if self.last_time is None:
            self.last_time = now
            return

        dt = now - self.last_time
        self.last_time = now

        rpm_mean = (wheel_msg.fl_rpm + wheel_msg.fr_rpm + wheel_msg.rl_rpm + wheel_msg.rr_rpm) / 4.0
        vx = rpm_mean * 2 * np.pi * self.R / 60.0

        delta = (wheel_msg.fl_steering_angle + wheel_msg.fr_steering_angle) / 2.0
        
        wz = vx / self.L * np.tan(delta)
        
        vy = 0.0

        #self.yaw += wz * dt

        r = R.from_quat([imu_msg.orientation.x, imu_msg.orientation.y, imu_msg.orientation.z, imu_msg.orientation.w])
        roll, pitch, yaw = r.as_euler('xyz', degrees=False)
        self.yaw = yaw

        self.x += vx * np.cos(self.yaw) * dt
        self.y += vx * np.sin(self.yaw) * dt

        odom = Odometry()
        odom.header = wheel_msg.header
        odom.header.frame_id = "fsds/odom"
        odom.child_frame_id = "fsds/FSCar"

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y

        #transforming yaw to quaternion
        
        odom.pose.pose.orientation.z = imu_msg.orientation.z
        odom.pose.pose.orientation.w = imu_msg.orientation.w

        odom.twist.twist.linear.x = vx
        odom.twist.twist.linear.y = vy
        odom.twist.twist.angular.z = wz

        odom.pose.covariance = [
            1.0,   0.0,   0.0,   0.0,   0.0,   0.0,
            0.0,   1.0,   0.0,   0.0,   0.0,   0.0,
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