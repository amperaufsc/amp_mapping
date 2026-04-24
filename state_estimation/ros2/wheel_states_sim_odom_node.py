#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from fs_msgs.msg import WheelStates
from message_filters import Subscriber, ApproximateTimeSynchronizer
from src.wheel_odometry import WheelOdometry

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

        self.wheel_odom = WheelOdometry()

    def wheel_callback(self, wheel_msg, imu_msg):
        odom = self.wheel_odom.wheelStatesToOdom(wheel_msg, imu_msg)

        if odom is not None:
            self.odom_pub.publish(odom)

def main(args=None):
    rclpy.init()
    wheel_rpm_odom = WheelRpmSimOdom()
    rclpy.spin(wheel_rpm_odom)
    wheel_rpm_odom.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()