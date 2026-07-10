#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_msgs.msg import TFMessage
from tf2_ros import TransformBroadcaster

class TFUnifier(Node):
    def __init__(self):
        super().__init__('tf_unifier')
        
        # Único publicador no /tf
        self.tf_broadcaster_ = TransformBroadcaster(self)
        
        # Subscreve às transformações do SLAM e odometria
        self.sub_slam = self.create_subscription(
            TFMessage, '/custom_slam_tf', self.slam_callback, 10)
        
        self.sub_odom = self.create_subscription(
            TFMessage, '/custom_odom_tf', self.odom_callback, 10)
        
        # Armazena as últimas transformações
        self.slam_transform_ = None
        self.odom_transform_ = None

    def slam_callback(self, msg):
        # Assume que o SLAM publica map→odom
        self.slam_transform_ = msg.transforms[0]
        self.publish_unified_tf()

    def odom_callback(self, msg):
        # Assume que a odometria publica odom→FSCar
        self.odom_transform_ = msg.transforms[0]
        self.publish_unified_tf()

    def publish_unified_tf(self):
        if self.slam_transform_ and self.odom_transform_:
            # Garante que os timestamps são iguais
            stamp = self.get_clock().now().to_msg()
            self.slam_transform_.header.stamp = stamp
            self.odom_transform_.header.stamp = stamp
            
            # Publica ambas transformações juntas
            self.tf_broadcaster_.sendTransform([
                self.slam_transform_,
                self.odom_transform_
            ])

def main(args=None):
    rclpy.init(args=args)
    node = TFUnifier()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()