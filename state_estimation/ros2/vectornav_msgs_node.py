#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from message_filters import Subscriber, ApproximateTimeSynchronizer
from sensor_msgs.msg import Imu
from sensor_msgs.msg import NavSatFix
from vectornav_msgs.msg import ImuGroup
from vectornav_msgs.msg import InsGroup
from vectornav_msgs.msg import AttitudeGroup


class VectornavMsgsNode(Node):
    def __init__(self):
        super().__init__('vectornav_msgs_node')

        self.imu_sub = Subscriber(self, ImuGroup, 'Imu_sub')
        self.ins_sub = self.create_subscription(InsGroup, 'Ins_sub', self.gps_callback, 10)
        self.attitude_sub = Subscriber(self, AttitudeGroup, 'Attitude_sub')

        self.imu_pub = self.create_publisher(Imu, 'imu', 10)
        self.gps_pub = self.create_publisher(NavSatFix, 'gps/fix', 10)

        queue_size = 10
        max_delay = 0.1
        
        self.time_sync = ApproximateTimeSynchronizer([self.imu_sub, self.attitude_sub], queue_size, max_delay)
        self.time_sync.registerCallback(self.imu_callback)
    

    def imu_callback(self, imu_msg, attitude_msg):
        imu = Imu()
        imu.header = imu_msg.header

        imu.orientation.x = attitude_msg.quaternion.y
        imu.orientation.y = attitude_msg.quaternion.x
        imu.orientation.z = - attitude_msg.quaternion.z 
        imu.orientation.w = attitude_msg.quaternion.w

        yaw_var = attitude_msg.ypru.x**2
        pitch_var = attitude_msg.ypru.y**2
        roll_var = attitude_msg.ypru.z**2
        imu.orientation_covariance = [yaw_var, 0.0, 0.0, 
                                      0.0, pitch_var, 0.0,
                                      0.0, 0.0, roll_var]

        imu.angular_velocity.x = imu_msg.angularrate.y
        imu.angular_velocity.y = imu_msg.angularrate.x
        imu.angular_velocity.z = - imu_msg.angularrate.z

        imu.angular_velocity_covariance = [1e-6, 0.0, 0.0,
                                           0.0, 1e-6, 0.0,
                                           0.0, 0.0, 1e-6]

        imu.linear_acceleration.x = imu_msg.accel.y
        imu.linear_acceleration.y = imu_msg.accel.x
        imu.linear_acceleration.z = - imu_msg.accel.z

        imu.linear_acceleration_covariance = [1e-4, 0.0, 0.0,
                                              0.0, 1e-4, 0.0,
                                              0.0, 0.0, 1e-4]

        self.imu_pub.publish(imu)

    def gps_callback(self, ins_msg):
        gps = NavSatFix()
        gps.header = ins_msg.header

        gps.latitude = ins_msg.poslla.x
        gps.longitude = ins_msg.poslla.y
        gps.altitude = ins_msg.poslla.z

        position_var = ins_msg.posu**2
        gps.position_covariance = [position_var, 0.0, 0.0,
                                   0.0, position_var, 0.0,
                                   0.0, 0.0, position_var]
        gps.position_covariance_type = 2

        self.gps_pub.publish(gps)


def main(args=None):
    rclpy.init()
    vectornav_msgs_node = VectornavMsgsNode()
    rclpy.spin(vectornav_msgs_node)
    vectornav_msgs_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()