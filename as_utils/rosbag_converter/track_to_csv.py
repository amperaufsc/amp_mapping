import rclpy
from rclpy.node import Node
from fs_msgs.msg import TrackStamped
from fs_msgs.msg import Cone  
import csv

class TrackToCSVNode(Node):
    def __init__(self):
        super().__init__('track_to_csv')
        self.subscriber = self.create_subscription(
            TrackStamped,
            '/track', 
            self.listener_callback,
            10
        )
        self.csv_file = open('track_data.csv', mode='w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow(['cone_id', 'x', 'y', 'z', 'color'])

    def listener_callback(self, msg):
        for i, cone in enumerate(msg.track):
            
            x = cone.location.x
            y = cone.location.y
            z = cone.location.z
            color = cone.color
            
            self.csv_writer.writerow([i, x, y, z, color])


def main(args=None):
    rclpy.init(args=args)
    node = TrackToCSVNode()
    rclpy.spin(node)
    node.csv_file.close()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
