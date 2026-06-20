#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped
import numpy as np
from fs_msgs.msg import ConeWithCovariance, Cone
from fs_msgs.msg import Track, TrackStamped,TrackStampedWithCovariance
from message_filters import Subscriber, ApproximateTimeSynchronizer
import rclpy.time
from src.mapper_function import LuisLopesMappingMethod
from src.map import Map
from src.obstacle import Obstacle
from src.vehicle_state import VehicleState
from nav_msgs.msg import Odometry
from tf2_ros.transform_listener import TransformListener
from tf2_ros.buffer import Buffer
from tf2_ros import LookupTransform



class MapperNode(Node):

    def __init__(self):
        super().__init__('mapper_node')
        self._tf_buffer=Buffer()
        self.track_sub = Subscriber(self,TrackStampedWithCovariance,"track")
        self.odom_sub = Subscriber(self,Odometry,"odom")
        self.tf_listener = TransformListener(self._tf_buffer,self)
        self.track_pub = self.create_publisher(Track, 'track_pub',10)
        self.target_frame = "fsds/map"
        self.source_frame = "fsds/FSCar"

        self.track_received = False
        self.first_pose = True
        self.first_track = True
        self.transform_received = False
        queue_size = 10
        max_delay = 1
        self.time_gap = 1       
        
        
        self.time_sync = ApproximateTimeSynchronizer([self.track_sub,self.odom_sub],queue_size,max_delay)
        self.time_sync.registerCallback(self.sync_callback)


    def sync_callback(self,track,odom):
        
        tempo = 0
        
        if tempo < self.time_gap:
            
            
            if self.transform_received:
                rotate_track =self.rotate_track(track,self.trans)

                track=self.array_to_track(rotate_track,track)
                self.track_to_obstacle(track)
                self.odom_to_state(odom)
                if self.first_pose:
                    self.mapper=LuisLopesMappingMethod(self.state)

                    self.first_pose = False

                self.odom_to_state(odom)

                self.map=self.mapper.update_map(self.obstacle_numpy_array,self.state,self.map)
                
                track = self.map_to_track(self.map.map)
                self.track_pub.publish(track)
            else:

                try:
                    self.trans=self._tf_buffer.lookup_transform(self.target_frame, self.source_frame, rclpy.time.Time()) #Use for simulation in FSDS
                    self.get_logger().info(self.trans.child_frame_id)
                    self.get_logger().warning("Transform received")
                    self.transform_received = True

                except:
                    self.get_logger().warning("Transform not received")
                
                    
            
    def array_to_track(self,array_track,track):
         
        for i in range(len(array_track)):
            cone_array=array_track[i][0]
            
            cone=cone_array[:3]
            
            x,y,z=cone
            
            track.track[i].location.x=float(x)
            track.track[i].location.y=float(y)
            track.track[i].location.z=float(z)
        return track
        


    def rotate_track(self,track,trans):
        T=self.transformation_matrix(trans)
        track_array=self.track_to_array(track.track)
        points = track_array

        points_transformed =np.array([[T @ point.T] for point in points])
        
        rotated_track = points_transformed.astype(np.float32)
        
        return rotated_track
        
    def transformation_matrix(self,trans):
        translaçao=trans.transform.translation
        rotaçao=trans.transform.rotation
        R = self.quaternion_to_rotation_matrix([rotaçao.x,rotaçao.y,rotaçao.z,rotaçao.w])[:3, :3]
        T = np.eye(4)
        t = np.array([translaçao.x,translaçao.y,translaçao.z])
        T[:3, :3] =R
        T[:3, 3] = t
        return T
    def quaternion_to_rotation_matrix(self,q):
    
        x, y, z, w = q

    # Normaliza o quaternion
        norm = np.sqrt(x*x + y*y + z*z + w*w)
        x /= norm
        y /= norm
        z /= norm
        w /= norm

    # Calcula a matriz de rotação
        R = np.array([
        [1 - 2*(y**2 + z**2),     2*(x*y - z*w),       2*(x*z + y*w)],
        [2*(x*y + z*w),           1 - 2*(x**2 + z**2), 2*(y*z - x*w)],
        [2*(x*z - y*w),           2*(y*z + x*w),       1 - 2*(x**2 + y**2)]
        ])

        return R
    

    def track_to_array(self,track):
        track_array = np.array([[cone.location.x,cone.location.y,cone.location.z,1] for cone in track])
        return track_array
    
            

    def track_to_obstacle(self, track):
        self.obstacle_numpy_array = []
        for cone in track.track:
            x = cone.location.x 
            y = cone.location.y 
            if cone.color == 0:
                color = 0
            elif cone.color == 1 or cone.color == 4:
                color = 4
            confidence = 0.7
            deviation = 0.3
            obstacle = Obstacle(x,y,confidence,color,deviation)
            self.obstacle_numpy_array.append(obstacle)
        if self.first_track:
            self.map=Map(self.obstacle_numpy_array)
            self.first_track=False
        
        
        self.track_received = True


    def odom_to_state(self,msg):
    
        
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        z = msg.pose.pose.position.z
        orientation_q = msg.pose.pose.orientation

        yaw = np.arctan2(2*(orientation_q.w*orientation_q.z - orientation_q.x*orientation_q.y), 1 - 2*((orientation_q.y)**2 + (orientation_q.z)**2))
        self.state=VehicleState(x,y,yaw,0,0,0)
    

    
    def map_to_track(self,map):
        
        track = Track()
        
        for obstacle in map:
            cone = Cone()
            cone.location.x=obstacle.x
            cone.location.y=obstacle.y
            cone.location.z=0.001
            if obstacle.label==0:
                cone.color = 0
            elif obstacle.label == 4 or obstacle.label == 1:
                cone.color = 1
            else:
                cone.color = 2
            track.track.append(cone)
        return track
            
            
    
def main():
    rclpy.init()
    mapper_node = MapperNode()
    rclpy.spin(mapper_node)
    mapper_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()