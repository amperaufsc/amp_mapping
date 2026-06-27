#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped
import numpy as np
from fs_msgs.msg import Cone, ConeWithCovariance
from fs_msgs.msg import TrackStampedWithCovariance,TrackStamped,Track
from message_filters import Subscriber, ApproximateTimeSynchronizer
import rclpy.time
from nav_msgs.msg import Odometry
from tf2_ros.transform_listener import TransformListener
from tf2_ros.buffer import Buffer
from tf2_ros import LookupTransform

class SimTrackNode(Node):

    def __init__(self):
        super().__init__('sim_mapper_node')
        self.track = TrackStampedWithCovariance()
        self._tf_buffer=Buffer()
        self.subscription = self.create_subscription(Odometry, 'odom', self.odom_callback, 10)
        self.subscription = self.create_subscription(Track, 'track', self.track_callback, 10)
        self.tf_listener = TransformListener(self._tf_buffer,self)
        self.track_pub = self.create_publisher(TrackStampedWithCovariance, 'track_pub',10)
        self.declare_parameter("apply_error",False)
        self.apply_error = self.get_parameter("apply_error").value
        self.raio = 30
        queue_size = 10
        max_delay = 1
        self.time_gap = 0.03
  

    def odom_callback(self,odom):
        nova_track = TrackStampedWithCovariance()
        nova_track.header = odom.header
        nova_track.header.frame_id = 'fsds/map'
        x = odom.pose.pose.position.x
        y = odom.pose.pose.position.y
        z = odom.pose.pose.position.z
        orientation_q = odom.pose.pose.orientation

        yaw = -np.arctan2(2*(orientation_q.w*orientation_q.z - orientation_q.x*orientation_q.y), 1 - 2*((orientation_q.y)**2 + (orientation_q.z)**2))
        transform_matrix = np.array([[np.cos(yaw),np.sin(yaw),x]
                                     ,[(-np.sin(yaw)),np.cos(yaw),y]
                                     ,[0,0,1]])
        inv_matrix = np.linalg.inv(transform_matrix)
        for cone in self.track.track:
            vetor_diferença = np.array(np.array((cone.location.x,cone.location.y,cone.location.z)) - np.array((x,y,z)))
            distancia = np.linalg.norm(vetor_diferença)

            if distancia <= self.raio:
                vector_cone = np.array([cone.location.x,cone.location.y,1]).T
                new_vector = inv_matrix @ vector_cone
                new_cone=ConeWithCovariance()
                new_cone.location.x = new_vector[0]
                new_cone.location.y = new_vector[1]
                new_cone.location.z = new_vector[2]
                new_cone.color = cone.color 
                nova_track.track.append(new_cone)
        if self.apply_error:
            nova_track = self.apply_error_on_track(nova_track)
        self.track_pub.publish(nova_track)
        
    
    def track_callback(self,msg):
        self.track = msg
    
    def apply_error_on_track(self,track):
        for cone in track.track:
            erro_angular = np.random.uniform(-np.pi,np.pi)
            erro_radial = np.random.normal(0,0.1)

            cone.location.x = cone.location.x + (np.cos(erro_angular) * erro_radial)
            cone.location.y = cone.location.y + (np.sin(erro_angular) * erro_radial)
        return track    
        
        
def main():
    rclpy.init()
    sim_track_node = SimTrackNode()
    rclpy.spin(sim_track_node)
    sim_track_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()