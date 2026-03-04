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


class VectornavOdomNode(Node):
    def __init__(self):
        super().__init__('vectornav_odom_node')

        self.imu_sub = Subscriber(self, ImuGroup, 'imu_sub')
        self.ins_sub = Subscriber(self, InsGroup, 'ins_sub')
        self.attitude_sub = Subscriber(self, AttitudeGroup, 'attitude_sub')

        self.odom_pub = self.create_publisher(Odometry, 'odom_pub', 10)

        queue_size = 10
        max_delay = 0.1

        self.time_sync = ApproximateTimeSynchronizer([self.imu_sub, self.ins_sub, self.attitude_sub], queue_size, max_delay)
        self.time_sync.registerCallback(self.sync_callback)
    
    def sync_callback(self, msg):
        


def main(args=None):
    rclpy.init()
    vectornav_odom_node = VectornavOdomNode()
    rclpy.spin(vectornav_odom_node)
    vectornav_odom_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()