# -*- coding: utf-8 -*-
import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from haru25_msgs.msg import CANArray

import sys
import termios

class RecordNode(Node):

    def __init__(self):
        super().__init__('record')
        self.can_sub_ = self.create_subscription(CANArray,'can/rx',self.can_cb, 10)
        self.timer = self.create_timer(0.01, self.timer_cb)
        self.i = 0
        self.id = 0

    def timer_cb(self):
        if self.i == 200:
            input()
    
    def can_cb(self, msg):
        print(msg)

def main(args=None):
    rclpy.init(args=args)
    node = RecordNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
