# -*- coding: utf-8 -*-
import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from sensor_msgs.msg import LaserScan

class TempNode(Node):

    def __init__(self):
        super().__init__('slam')
        self.str_sub_ = self.create_subscription(LaserScan,'/scan',self.scan_cb, 10)
        
    def scan_cb(self, msg):
        angle_min = msg.angle_min
        angle_max = msg.angle_max
        

def main(args=None):
    rclpy.init(args=args)
    node = TempNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
