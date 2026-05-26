# -*- coding: utf-8 -*-
import rclpy
from rclpy.node import Node

from std_msgs.msg import String

class TempNode(Node):

    def __init__(self):
        super().__init__('temp')
        self.str_pub_ = self.create_publisher(String, 'tx', 10)
        self.str_sub_ = self.create_subscription(String,'rx',self.str_cb, 10)
        timer_period = 0.  # seconds
        self.timer = self.create_timer(timer_period, self.timer_cb)
        self.i = 0

    def timer_cb(self):
        msg = String()
        msg.data = 'Hello World: %d' % self.i
        self.str_pub_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1
    
    def str_cb(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)

def main(args=None):
    rclpy.init(args=args)
    node = TempNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()