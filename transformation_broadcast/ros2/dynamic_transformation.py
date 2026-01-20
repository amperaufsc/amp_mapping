#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_msgs.msg import TFMessage
from nav_msgs.msg import Odometry
import time

class DynamicBroadcaster(Node):
    def __init__(self):
        super().__init__('dynamic_broadcaster')
        self.name_ = "fsds/FSCar"
        
        # Publisher para o tópico customizado (não mais /tf direto)
        self.tf_publisher_ = self.create_publisher(
            TFMessage,
            '/tf',  # Tópico customizado para odometria
            10
        )
        
        # Subscreve ao tópico de odometria original
        self.sub_odom = self.create_subscription(
            Odometry,
            "odom",
            self.handle_odom,
            10
        )
        
        self.get_logger().info("Broadcasting odometry to /tf")

    def handle_odom(self, msg):
        # Cria a mensagem de transformação
        tfs = TransformStamped()
        tfs.header.stamp = msg.header.stamp
        tfs.header.frame_id = "fsds/odom"
        tfs.child_frame_id = self.name_

        # Preenche os dados de posição e orientação
        tfs.transform.translation.x = msg.pose.pose.position.x
        tfs.transform.translation.y = msg.pose.pose.position.y
        tfs.transform.translation.z = msg.pose.pose.position.z
        tfs.transform.rotation = msg.pose.pose.orientation

        # Empacota em uma mensagem TFMessage e publica no tópico customizado
        tf_msg = TFMessage()
        tf_msg.transforms = [tfs]
        self.tf_publisher_.publish(tf_msg)

def main(args=None):
    rclpy.init(args=args)
    node = DynamicBroadcaster()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()