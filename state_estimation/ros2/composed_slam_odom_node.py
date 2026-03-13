#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from message_filters import Subscriber, ApproximateTimeSynchronizer
from vectornav_msgs.msg import ImuGroup, InsGroup, AttitudeGroup
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped


class EkfOdomNode(Node):
    def __init__(self):
        super().__init__('composed_slam_odom_node')

        self.pose_sub = Subscriber(self, PoseWithCovarianceStamped, 'pose_sub')
        self.imu_sub = Subscriber(self, ImuGroup, 'Imu_sub')
        self.ins_sub = Subscriber(self, InsGroup, 'Ins_sub')
        self.attitude_sub = Subscriber(self, AttitudeGroup, 'Attitude_sub')

        self.odom_pub = self.create_publisher(Odometry, 'odom_pub', 10)
        
        queue_size = 10
        max_delay = 0.1
        
        self.time_sync = ApproximateTimeSynchronizer([self.imu_sub, self.ins_sub, self.attitude_sub, self.pose_sub], queue_size, max_delay)
        self.time_sync.registerCallback(self.odom_callback)
    

    def odom_callback(self, imu_msg, ins_msg, attitude_msg, pose_msg):
        odom = Odometry()
        odom.header = attitude_msg.header
        odom.header.frame_id = 'base_link'

        odom.pose.pose.position.x = pose_msg.pose.pose.position.x
        odom.pose.pose.position.y = pose_msg.pose.pose.position.y
        odom.pose.pose.position.z = pose_msg.pose.pose.position.z

        odom.pose.pose.orientation.x = attitude_msg.quaternion.y
        odom.pose.pose.orientation.y = attitude_msg.quaternion.x
        odom.pose.pose.orientation.z = - attitude_msg.quaternion.z
        odom.pose.pose.orientation.w = attitude_msg.quaternion.w

        odom.twist.twist.linear.x = ins_msg.velbody.x
        odom.twist.twist.linear.y = - ins_msg.velbody.y
        odom.twist.twist.linear.z = - ins_msg.velbody.z

        odom.twist.twist.angular.x = imu_msg.angularrate.x
        odom.twist.twist.angular.y = - imu_msg.angularrate.y
        odom.twist.twist.angular.z = - imu_msg.angularrate.z

        self.odom_pub.publish(odom)


def main(args=None):
    rclpy.init()
    ekf_odom_node = EkfOdomNode()
    rclpy.spin(ekf_odom_node)
    ekf_odom_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()