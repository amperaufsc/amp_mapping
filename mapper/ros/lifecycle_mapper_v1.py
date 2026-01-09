#!/usr/bin/env python3
import rclpy
from rclpy.lifecycle import LifecycleNode
from rclpy.lifecycle import State
from rclpy.lifecycle import TransitionCallbackReturn
from rclpy.node import Node
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped
import numpy as np
from std_msgs.msg import String
from fs_msgs.msg import Track
from fs_msgs.msg import ConeWithCovariance, Cone
from fs_msgs.msg import TrackStampedWithCovariance, TrackStamped
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
from rclpy.lifecycle import LifecycleNode, LifecycleState, TransitionCallbackReturn
from lifecycle_msgs.srv import GetState, ChangeState
from lifecycle_msgs.msg import Transition, TransitionEvent
import time
import asyncio
import os
import sys
import time



class MapperNode(LifecycleNode):

    def __init__(self):
        super().__init__('mapper_node')
        self._tf_buffer=Buffer()        
        self.track_received = False
        self.first_pose=True
        self.first_track=True
        self.transform_received=False
        self.queue_size = 10
        self.max_delay = 1
        self.time_gap = 1
        self.get_logger().warning("construi")
                
    def on_cleanup(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info('IN on_cleanup')
        if hasattr(self, 'track_pub'):
            self.destroy_lifecycle_publisher(self.track_pub)

        if hasattr(self, 'time_sync'):
            self.time_sync.callbacks.clear()
            
        return TransitionCallbackReturn.SUCCESS   
    
    def on_shutdown(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info("Shutdown MAPPER.")
        if hasattr(self, 'track_pub'):
            self.destroy_lifecycle_publisher(self.track_pub)

        if hasattr(self, 'time_sync'):
            self.time_sync.callbacks.clear() 

        return TransitionCallbackReturn.SUCCESS    
    
    def on_configure(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info('Configuring MapperNode...')
        self.track_sub = Subscriber(self,TrackStampedWithCovariance,"/track")
        self.odom_sub = Subscriber(self,Odometry,"/orbslam/odom")
        self.tf_listener = TransformListener(self._tf_buffer,self)
        self.track_pub = self.create_publisher(Track, '/track_pub', 10)

        #alterador do estado do state_machine_as
        self.transition_event_pub = self.create_publisher(TransitionEvent, 'EbsNOTMissionFinished', 10) 
        self.led_pub = self.create_publisher(String, '/AMP/as_status_indicator', 10)
        self.led_msg = String()
        self.get_logger().info('Configuração MAPPER concluída com sucesso.')
        return TransitionCallbackReturn.SUCCESS

        
    
    def on_activate(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info('Activating MapperNode...')
        self.is_activated = True
        self.time_sync = ApproximateTimeSynchronizer([self.track_sub,self.odom_sub],self.queue_size,self.max_delay)
        self.time_sync.registerCallback(self.sync_callback)
        return super().on_activate(state)
    
    def on_deactivate(self, state):
        self.get_logger().info("Into Deactivate MAPPER")
        self.is_activated = False
        return super().on_deactivate(state)

    def sync_callback(self,track,odom):
        self.get_logger().warning("entrei no callbackk")
        
        tempo = 0
        
        if tempo < self.time_gap:
            self.get_logger().warning("tempo ta bom")
            
            
            if self.transform_received:
                self.get_logger().warning("transformei")
                
                
                
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
                    self.trans=self._tf_buffer.lookup_transform("left_camera_link","oak_left_camera_optical_frame",rclpy.time.Time())
                    self.get_logger().info(self.trans.child_frame_id)
                    self.get_logger().warning("Deu bom")
                    self.transform_received = True

                except:
                    # Ele vai publicar um evento pro state_machine, ao mesmo tempo que ele
                    # ira setar o shutdown do lifecyle na funcao do launch. Caso a transicao via lifecycle
                    # (a do launch) falhe, a principio ele garante pelo menos que o AS_Emergency sera 
                    # ativado no state_machine_as
                    #x=1/0 # essa merda aqui é so pra forçar que o nó morra
                    self.get_logger().warning("Deu merda")
                    event = TransitionEvent()
                    #self.change_own_state(Transition.TRANSITION_ACTIVE_SHUTDOWN) # -> Isso aqui por enquanto so ta implementado
                    self.transition_event_pub.publish(event)          # no mapper. Ele tenta dar shutdown pela
                                                                                # pela funcao, evitando matar o nó para forçar 
                                                                                #ele renascer para entrar em shutdown, o problema
                    # Publica o sinal para ativacao do led                  # é que ele nao funciona quase nunca e pra cada
                    self.led_msg.data = 'as_emergency'                      # requisicao ele espera por 2 segundos entao atrasa
                    self.led_pub.publish(self.led_msg)                      # tanto, e acredito que mais, que matar e renascer
                                                                                # igual é feito no launch
                    sys.exit(1)
                
                    
            
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
        
        
        
    
        rotated_track = points_transformed[:3].astype(np.float32)
        
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
            else:
                color = 4
            confidence = cone.confidence
            deviation = cone.deviation
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
            cone = ConeWithCovariance()
            cone.location.x=obstacle.x
            cone.location.y=obstacle.y
            cone.location.z=0.001
            if obstacle.label==0:
                cone.color = 0
            elif obstacle.label == 4:
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