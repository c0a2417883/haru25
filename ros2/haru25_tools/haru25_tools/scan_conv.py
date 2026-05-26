# -*- coding: utf-8 -*-
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from haru25_msgs.msg import CANArray
from std_msgs.msg import Float32
import time
import math

class TempNode(Node):

    def __init__(self):
        super().__init__('scan_conv')
        self.str_pub_ = self.create_publisher(LaserScan, 'scan/conv', 10)
        self.wheel_pub_ = []
        for i in range(4):
            self.wheel_pub_.append(self.create_publisher(Float32, f'error/wheel{i+1}', 10))
        self.marker_pub_ = self.create_publisher(Marker, 'wall', 10)

        
        self.str_sub_ = self.create_subscription(LaserScan,'scan',self.scan_cb, 10)
        self.can_sub_ = self.create_subscription(CANArray,'can/rx',self.can_cb, 10)
        
        time.sleep(1)
        self.msg = Marker()
        self.msg.header.frame_id = 'map'
        self.msg.header.stamp = self.get_clock().now().to_msg()
        self.msg.type = 5
        self.msg.action = 0
        self.msg.color.a = self.msg.color.r = 1.0 #RED
        self.msg.scale.x = 0.039
        for i in range(2):
            self.flip = -1.0 if i ==1 else 1.0
            self.addWall(0.038, 3.462, -0.019, -0.019)
            self.addWall(0.038, 3.462, 6.943, 6.943)
            self.addWall(1.526, 2.526, 1.481, 1.481)
            self.addWall(0.626, 1.526, 2.481, 2.481)
            self.addWall(0.038, 0.588, 2.981, 2.981)
            self.addWall(1.526, 2.526, 3.481, 3.481)
            self.addWall(0.626, 1.526, 4.481, 4.481)
            self.addWall(1.526, 2.526, 5.481, 5.481)
            self.addWall(1.526, 2.526, 3.481, 3.481)
    
            self.addWall(0.019, 0.019, -0.038, 6.962)
            self.addWall(3.481, 3.481, -0.038, 6.962)
            self.addWall(0.607, 0.607, 1.462, 5.5)
            self.addWall(2.545, 2.545, 1.462, 5.5)
        
        self.get_logger().info('publish wall')
        self.marker_pub_.publish(self.msg)
    
    def scan_cb(self, msg:LaserScan):
        msg.header.stamp = self.get_clock().now().to_msg() 
        self.str_pub_.publish(msg)
        
    def addWall(self, x1, x2, y1, y2):
        p1 = Point()
        p1.x = x1 * self.flip
        p1.y = y1
        p1.z = 0.0
        p2 = Point()
        p2.x = x2 * self.flip
        p2.y = y2
        p2.z = 0.0
        self.msg.points.append(p1)
        self.msg.points.append(p2)
    
    def can_cb(self, msg:CANArray):
        flt = Float32()
        for can in msg.array:
            if can.id < 4 and len(can.data) == 6:
                flt.data = abs(can.data[3] - can.data[2]) / (2.*math.pi)*60  # 出力速度 - 入力速度
                flt.data = flt.data
                self.wheel_pub_[can.id].publish(flt)
        

def main(args=None):
    rclpy.init(args=args)
    node = TempNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
