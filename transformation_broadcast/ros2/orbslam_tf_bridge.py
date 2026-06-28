#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster, TFMessage

class OrbslamTfBridge(Node):
    def __init__(self):
        super().__init__('orbslam_tf_bridge')
        
        self.tf_broadcaster = TransformBroadcaster(self)
        
        self.subscription = self.create_subscription(TransformStamped,'/AMP/orbslam/transform',self.listener_callback,10)
        
        self.get_logger().info("Ponte TF iniciada! Escutando /orbslam/transform e convertendo para /tf")

    def listener_callback(self, msg: TransformStamped):
        msg.header.frame_id = 'map'
        msg.child_frame_id = 'left_camera_link'
        
        self.tf_broadcaster.sendTransform(msg)

def main(args=None):
    rclpy.init()
    orbslam_tf_bridge = OrbslamTfBridge()
    rclpy.spin(orbslam_tf_bridge)
    orbslam_tf_bridge.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()