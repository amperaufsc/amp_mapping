#!/usr/bin/env python3
import rclpy
from rclpy.lifecycle import LifecycleNode
from rclpy.lifecycle import LifecycleState
from rclpy.lifecycle import TransitionCallbackReturn
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



class MapperNode(LifecycleNode):

    def __init__(self):
        super().__init__('mapper_node_life')

        #Reservando espaço pra usar depois
        self.get_logger().info('Mapper Unconfigured. (o_o)')
        self._tf_buffer = None
        self.tf_listener = None
        self.track_sub = None
        self.odom_sub = None
        self.time_sync = None
        self.track_pub = None

        self.declare_parameter('target_frame', 'fsds/map')
        self.declare_parameter('source_frame', 'fsds/FSCar')
        self.declare_parameter('queue_size', 10)
        self.declare_parameter('max_delay', 1.0)
        self.declare_parameter('time_gap', 1)


    #Lifecycle Callbacks

    def on_configure(self, state: LifecycleState) -> TransitionCallbackReturn:
        #Escuta para ter as informações mas não publica
        self.get_logger().info('Configuring MapperNode...')

        try:
            self._tf_buffer = Buffer()
            self.track_sub = Subscriber(self, TrackStampedWithCovariance, "/fsds/testing_only/track")
            # Normal odom
            #self.odom_sub = Subscriber(self, Odometry, "odom")
            self.odom_sub = Subscriber(self, Odometry, "/fsds/testing_only/odom")
            self.tf_listener = TransformListener(self._tf_buffer, self)
            self.track_pub = self.create_lifecycle_publisher(Track, 'track_pub', 10)

            self.target_frame = self.get_parameter('target_frame').value
            self.source_frame = self.get_parameter('source_frame').value
            queue_size = self.get_parameter('queue_size').value
            max_delay = self.get_parameter('max_delay').value
            self.time_gap = self.get_parameter('time_gap').value

            self.track_received = False
            self.first_pose = True
            self.first_track = True
            self.transform_received = False

            self.time_sync = ApproximateTimeSynchronizer([self.track_sub, self.odom_sub], queue_size, max_delay)
            self.time_sync.registerCallback(self.sync_callback)

            self.get_logger().info('MapperNode Configured! (o.o)')
            return TransitionCallbackReturn.SUCCESS
        except Exception as e:
            self.get_logger().error(f"Configuration failed: {e}")
            return TransitionCallbackReturn.ERROR


    def on_activate(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info('Activating MapperNode...')

        try:
            self.track_pub.on_activate(state)

            self.get_logger().info('MapperNode Activated! (o‿o)')
            return super().on_activate(state)

        except Exception as e:
            self.get_logger().error(f"Activation failed: {e}")
            return TransitionCallbackReturn.ERROR


    def on_deactivate(self, state: LifecycleState) -> TransitionCallbackReturn:
        #Contrário do on_activate
        self.get_logger().info('Deactivating MapperNode...')

        try:
            self.track_pub.on_deactivate(state)

            self.get_logger().info('MapperNode Deactivated! (-‿-)')
            return super().on_deactivate(state)

        except Exception as e:
            self.get_logger().error(f"Deactivation failed: {e}")
            return TransitionCallbackReturn.ERROR


    def on_cleanup(self, state: LifecycleState) -> TransitionCallbackReturn:
        # Terminar de "limpar" o nó
        self.get_logger().info('Cleaning up MapperNode...')

        try:
            self._destroy_subscriptions()
            self._destroy_publishers()

            self.tf_listener = None
            self._tf_buffer = None
            self.time_sync = None

            self.track_received = False
            self.first_pose = True
            self.first_track = True
            self.transform_received = False

            self.get_logger().info('MapperNode Cleaned Up! (x‿x)')
            return TransitionCallbackReturn.SUCCESS
        except Exception as e:
            self.get_logger().error(f"Cleaning failed: {e}")
            return TransitionCallbackReturn.ERROR


    def on_shutdown(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info('Shutting down MapperNode...')

        try:
            self._destroy_subscriptions()
            self._destroy_publishers()

            self.tf_listener = None
            self._tf_buffer = None
            self.time_sync = None

            self.get_logger().info('MapperNode Shutted Down! (x_x)')
            return TransitionCallbackReturn.SUCCESS
        except Exception as e:
            self.get_logger().error(f"Shutdown failed: {e}")
            return TransitionCallbackReturn.ERROR


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

    #Função Própria
    def _destroy_publishers(self, names=None):
        if names is None:
            names = [
                'track_pub',
            ]

        for name in names:
            pub = getattr(self, name, None)

            if pub is not None:
                self.destroy_lifecycle_publisher(pub)
                setattr(self, name, None)


    def _destroy_subscriptions(self, names=None):
        if names is None:
            names = [
                'track_sub',
                'odom_sub',
            ]

        for name in names:
            sub = getattr(self, name, None)

            if sub is not None:
                self.destroy_subscription(sub)
                setattr(self, name, None)


def main():
    rclpy.init()
    mapper_node = MapperNode()
    rclpy.spin(mapper_node)
    mapper_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()