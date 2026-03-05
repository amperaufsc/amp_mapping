#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped
from fs_msgs.msg import WheelStates
from sensor_msgs.msg import Imu
from message_filters import Subscriber, ApproximateTimeSynchronizer
from vectornav_msgs.msg import AttitudeGroup, ImuGroup, InsGroup
import numpy as np
import pymap3d as pm
from src.coordinate_transforms import CoordinateTransforms

class VectornavOdomNode(Node):
    def __init__(self):
        super().__init__('vectornav_odom_node')

        self.imu_sub = Subscriber(self, ImuGroup, 'imu_sub')
        self.ins_sub = Subscriber(self, InsGroup, 'ins_sub')
        self.attitude_sub = Subscriber(self, AttitudeGroup, 'attitude_sub')

        self.odom_pub = self.create_publisher(Odometry, 'odom_pub', 10)

        queue_size = 10
        max_delay = 0.1
        self.first_gps = True
        self.first_lla = None

        self.time_sync = ApproximateTimeSynchronizer([self.imu_sub, self.ins_sub, self.attitude_sub], queue_size, max_delay)
        self.time_sync.registerCallback(self.sync_callback)
    
    def sync_callback(self, imu_msg, ins_msg, attitude_msg):
        odom = Odometry()

        if self.first_gps:
            self.first_gps = False
            lat = ins_msg.poslla.x
            lon = ins_msg.poslla.y
            alt = ins_msg.poslla.z
            self.first_lla = CoordinateTransforms(lat, lon, alt)

        x_ecef = ins_msg.posecef.x
        y_ecef = ins_msg.posecef.y
        z_ecef = ins_msg.posecef.z

        east, north, up = self.first_lla.ecefToEnu(x_ecef, y_ecef, z_ecef)

        odom.pose.pose.position.x = east
        odom.pose.pose.position.y = north
        odom.pose.pose.position.z = up


def main(args=None):
    rclpy.init()
    vectornav_odom_node = VectornavOdomNode()
    rclpy.spin(vectornav_odom_node)
    vectornav_odom_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()