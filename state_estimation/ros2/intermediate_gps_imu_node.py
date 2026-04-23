#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from sensor_msgs.msg import NavSatFix
import numpy as np

class IntermediateGpsImuNode(Node):
    def __init__(self):
        super().__init__('intermediate_gps_imu_node')

        self.gps_sub = self.create_subscription(NavSatFix, '/fsds/gps', self.gps_callback, 10)
        self.imu_sub = self.create_subscription(Imu, '/fsds/imu', self.imu_callback, 10)

        self.gps_pub = self.create_publisher(NavSatFix, 'gps/fix', 10)
        self.imu_pub = self.create_publisher(Imu, 'imu', 10)

    def gps_callback(self, msg):
        gps = NavSatFix()
        gps = msg

        self.gps_pub.publish(gps)

    def imu_callback(self, msg):
        imu = Imu()
        imu = msg

        self.imu_pub.publish(imu)


def main(args=None):
    rclpy.init()
    intermediate_gps_imu = IntermediateGpsImuNode()
    rclpy.spin(intermediate_gps_imu)
    intermediate_gps_imu.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()