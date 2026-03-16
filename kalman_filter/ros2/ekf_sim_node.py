#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64
from error_space_kalman_filter.error_space_kalman_filter import Error_Space_Kalman_Filter
import numpy as np

#imu sempre que receber a msg
#dvl e slam chamar de acordo com quantas vezes teve a imu

# O QUE EU PRECISO:
# aceleração - IMU            --> imu msg 
# velocidade angular - IMU    --> imu msg
# posição - SLAM              --> odom msg
# velocidade - DVL            --> odom msg 

class KalmanFilterNode(Node):
    def __init__(self):
        super().__init__('ekf_sim_node')
        self.subscription = self.create_subscription(Odometry, '/fsds/testing_only/odom', self.odom_callback, 10)
        self.subscription = self.create_subscription(Imu, '/fsds/imu', self.imu_callback, 10)
        self.ekf_odom_pub = self.create_publisher(Odometry, '/ekf_odometry',10)
        self.pub_freq = self.create_publisher(Float64, '/freq_ekf',10)

        self.imu = np.zeros((6,1))
        self.dvl = np.zeros((3,1))
        self.slam = np.zeros((6,1))

        self.declare_parameter('imu_update_time', 0.01)
        self.declare_parameter('dvl_update_time', 0.05)
        self.declare_parameter('slam_update_time', 0.1)
        self.declare_parameter('accelerometer_random_walk_bias', [0.00003745251231312492,0.00003745251231312492,0.00003745251231312492])
        self.declare_parameter('gyroscope_random_walk_bias', [0.0000005005899751017297,0.0000005005899751017297,0.0000005005899751017297])
        self.declare_parameter('accelerometer_noise', [0.0017862283519383298,0.0017862283519383298,0.0017862283519383298])
        self.declare_parameter('gyroscope_noise', [0.00010030383677353103,0.00010030383677353103,0.00010030383677353103])
        self.declare_parameter('corr_noise', [0.001,0.001,0.001])
        self.declare_parameter('dvl_noise', [0.0001,0.0001,0.0001])
        self.declare_parameter('slam_noise', [0.01,0.01,0.01,0.0001,0.0001,0.0001])
        self.declare_parameter('corr_t', [0.09, 0.09, 0.09])

        self.imu_update_time = self.get_parameter('imu_update_time').value
        self.dvl_update_time = self.get_parameter('dvl_update_time').value
        self.slam_update_time = self.get_parameter('slam_update_time').value
        self.accelerometer_random_walk_bias = self.get_parameter('accelerometer_random_walk_bias').value
        self.gyroscope_random_walk_bias = self.get_parameter('gyroscope_random_walk_bias').value
        self.accelerometer_noise = self.get_parameter('accelerometer_noise').value
        self.gyroscope_noise = self.get_parameter('gyroscope_noise').value
        self.corr_noise = self.get_parameter('corr_noise').value
        self.dvl_noise = self.get_parameter('dvl_noise').value
        self.slam_noise = self.get_parameter('slam_noise').value
        self.corr_t = self.get_parameter('corr_t').value

        self.timer = self.create_timer(float(self.imu_update_time), self.timer_callback)
        self.data = Odometry()
        self.freq = Float64()
        self.last_timestamp = 0


        #temos problemas com esses valores de inicialização da classe
        self.kalman_filter = Error_Space_Kalman_Filter(
                                                        self.imu_update_time,
                                                        self.dvl_update_time,
                                                        self.slam_update_time,
                                                        np.asarray(self.accelerometer_random_walk_bias).reshape(-1,1),
                                                        np.asarray(self.gyroscope_random_walk_bias).reshape(-1,1),
                                                        np.asarray(self.accelerometer_noise).reshape(-1,1),
                                                        np.asarray(self.gyroscope_noise).reshape(-1,1),
                                                        np.asarray(self.corr_noise).reshape(-1,1),
                                                        np.asarray(self.dvl_noise).reshape(-1,1),
                                                        np.asarray(self.slam_noise).reshape(-1,1),
                                                        np.asarray(self.corr_t).reshape(-1,1))


    def imu_callback(self, msg):
        imu_accel =   msg.linear_acceleration
        imu_ang_vel = msg.angular_velocity
        self.imu = np.array([[imu_accel.x],
                            [imu_accel.y],
                            [imu_accel.z],
                            [imu_ang_vel.x],
                            [imu_ang_vel.y],
                            [imu_ang_vel.z]])


    def odom_callback(self, msg):
        self.data.header = msg.header
        
        pos = msg.pose.pose.position
        vel = msg.twist.twist.linear
        ori = msg.pose.pose.orientation

        self.dvl = np.array([[vel.x],
                             [vel.y],
                             [vel.z]])
        self.slam = np.array([[pos.x],
                              [pos.y],
                              [pos.z],
                              [ori.x],
                              [ori.y],
                              [ori.z]])


    def timer_callback(self):
        filter = self.kalman_filter.ekf_principal(self.imu, self.dvl, self.slam)

        self.data.pose.pose.position.x = float(filter[0][0])
        self.data.pose.pose.position.y = float(filter[0][1])
        self.data.pose.pose.position.z = float(filter[0][2])
        self.data.twist.twist.linear.x = float(filter[1][0])
        self.data.twist.twist.linear.y = float(filter[1][1])
        self.data.twist.twist.linear.z = float(filter[1][2])
        self.data.pose.pose.orientation.x = float(filter[2][0])
        self.data.pose.pose.orientation.y = float(filter[2][1])
        self.data.pose.pose.orientation.z = float(filter[2][2])

        self.ekf_odom_pub.publish(self.data)

        self.timestamp = self.get_clock().now().to_msg()
        t = self.timestamp.sec + self.timestamp.nanosec * 1e-9
        self.freq.data = 1 / (t - self.last_timestamp)
        self.pub_freq.publish(self.freq)
        self.last_timestamp = t


def main(args=None):
    rclpy.init(args=args)
    kalman_filter_node = KalmanFilterNode()
    rclpy.spin(kalman_filter_node)
    kalman_filter_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()