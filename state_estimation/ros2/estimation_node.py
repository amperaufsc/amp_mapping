#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped
from fs_msgs.msg import WheelStates
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32
import numpy as np
from transforms3d.euler import quat2euler, euler2quat


class EstimationNode(Node):
    def __init__(self):
        super().__init__('estimation_node')

        self.odom_subscription = self.create_subscription(Odometry, 'odom', self.odom_callback, 10)
        self.imu_subscription = self.create_subscription(Imu, 'imu', self.imu_callback, 10)
        self.wheel_subscription = self.create_subscription(WheelStates, 'wheel_states', self.wheel_callback, 10)
    

        self.odom_publisher = self.create_publisher(Odometry, 'propagated_odom', 10)
        self.pose_publisher = self.create_publisher(Pose, 'pose',10)
        self.prop_pose_publisher = self.create_publisher(Pose, 'prop_pose',10)

        self.T1 = 0.04
        self.timer1 = self.create_timer(self.T1, self.timer1_callback)

        self.T2 = 0.1
        self.timer1 = self.create_timer(self.T2, self.timer2_callback)

        self.get_logger().info('Estimator started')
        self.odom_received = False
        self.imu_received = False
        self.wheel_received = False
        
    
    def odom_callback(self, msg):
        self.odom_msg = msg
        self.odom_received = True
        self.pose_publisher.publish(msg.pose.pose)

    def imu_callback(self, msg):
        self.imu_msg = msg
        self.imu_received = True

    def wheel_callback(self, msg):
        self.wheel_msg = msg
        self.wheel_received = True

    def timer1_callback(self):
        if self.imu_received and self.odom_received and self.wheel_received:
            self.imu_received = False
            self.wheel_received = False
            
            q_x, q_y, q_z, q_w = self.imu_msg.orientation
            roll, pitch, yaw = quat2euler([ q_w, q_x, q_y, q_z])
            
            speed_avarage = (self.wheel_msg.rl_rpm + self.wheel_msg.rr_rpm)/2

            self.odom_ref.pose.pose.position.x = np.cos(yaw)*speed_avarage*self.T1 + self.odom_ref.pose.pose.position.x
            self.odom_ref.pose.pose.position.y = np.sin(yaw)*speed_avarage*self.T1 + self.odom_ref.pose.pose.position.y

            self.odom_ref.pose.pose.orientation = euler2quat(roll,pitch,yaw)

            self.odom_ref.twist.twist.linear.x = np.cos(yaw)*speed_avarage
            self.odom_ref.twist.twist.linear.y = np.sin(yaw)*speed_avarage

            self.odom_publisher(self.odom_ref)
            self.prop_pose_publisher(self.odom_ref.pose.pose)


        if not self.imu_received:
            self.get_logger().warn('No new imu received')
        if not self.wheel_received:
            self.get_logger().warn('No new wheel states received')

    def timer2_callback(self):
        if self.odom_received:
            self.odom_ref = self.odom_msg
            self.odom_received = False
        else:
            self.get_logger().warn('No odometry received')

def main(args=None):
    rclpy.init()
    estimation_node = EstimationNode()
    rclpy.spin(estimation_node)
    estimation_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()