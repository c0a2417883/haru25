# -*- coding: utf-8 -*-
import struct
from math import pi
import numpy as np

# ROS
import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32MultiArray
from can_msgs.msg import Frame

dt = 0.005  # seconds

def constrain(a, b, c):
    if(a > c):
        return c
    elif(a < b):
        return b
    return a

class PI():
    def __init__(self, gain_p, gain_i):
        self.gain_p = gain_p
        self.gain_i = gain_i
        self.integral = 0

    def update(self, error, limit):
        out = error * self.gain_p
        out = constrain(out, -limit, limit);#outが、-max_とmax_との間にあるときはout、outが-max_より小さいときは-max_、outがmax_より大きいときはmax_を返す
        self.integral += self.gain_i * error * dt
        windup = limit - abs(out)#Anti-windup
        self.integral = constrain(self.integral, -limit, limit) if (windup > 0.) else 0.
        out += self.integral
        return out

class Robomas():
    rx_data = []
    m2006 = True
    state_pos = 0.
    state_vel = 0.
    state_curr = 0.
    prev_pos = 0.
    rotate = 0
    pos_p = PI(2., 0)
    vel_pi = PI(0.1, 2.)
    def update(self):
        rx_data = self.rx_data
        l = len(rx_data)
        ref_current = 0
        if l <= 1:#停止
            ref_current = 0.
        elif l <= 2:#電流制御
            ref_current =  rx_data[1]
        elif l <= 3:#速度制御
            ref_current =  self.vel_pi.update(rx_data[1] - self.state_vel, rx_data[2])
        elif l <= 4:#位置制御
            ref_vel = self.pos_p.update(rx_data[1] - self.state_pos, rx_data[2])
            ref_current = self.vel_pi.update(ref_vel - self.state_vel, rx_data[2])
        if self.m2006:
            ref_current  = constrain(ref_current,-10.,10.)
            ref_current *= 10000. / 10.
        else:
            ref_current  = constrain(ref_current,-20.,20.)
            ref_current *= 16384. / 20.
        return int(ref_current)

class TempNode(Node):

    def __init__(self):
        super().__init__('robomas_control')
        # GUI
        self.robomas_pub_ = self.create_publisher(Float32MultiArray, 'robomas/rx', 10)
        self.robomas_sub_ = self.create_subscription(Float32MultiArray,'robomas/tx',self.robomas_cb, 10)
        
        # SocketCAN
        self.can_pub_ = self.create_publisher(Frame, '/to_can_bus', 10)
        self.can_sub_ = self.create_subscription(Frame,'/from_can_bus',self.can_cb, 10)
        
        # PI
        self.robomas = Robomas()
        
        self.timer = self.create_timer(dt, self.timer_cb)

    def timer_cb(self):
        msg = Frame()
        msg.id = 0x200
        msg.dlc = 8
        ref_current = self.robomas.update()
        ref_current = [ref_current, 0, 0, 0] 
        #print(ref_current)
    
        #SEND
        ref = struct.pack(f'>4h', *ref_current)
        for i in range(8):
            msg.data[i] = ref[i]
        self.can_pub_.publish(msg)

        msg = Float32MultiArray()
        msg.data.append(0)
        msg.data.append(self.robomas.state_pos)
        msg.data.append(self.robomas.state_vel)
        msg.data.append(self.robomas.state_curr)
        self.robomas_pub_.publish(msg)
    
    def robomas_cb(self, msg):
        if len(msg.data) == 0:
            return
        if msg.data[0] == 0:
            self.robomas.rx_data = msg.data[:]
    
    def can_cb(self, msg):
        if msg.id == 0x200 + 1:
            data = struct.unpack('>hhhh', msg.data)
            if self.robomas.m2006:
                if(data[0] - self.robomas.prev_pos > 4096):
                    self.robomas.rotate -= 1
                elif (data[0] - self.robomas.prev_pos < -4096):
                    self.robomas.rotate += 1
                self.robomas.prev_pos = data[0]
                self.robomas.state_pos = (float(data[0]) / 8192. + self.robomas.rotate)*pi*2. / 36.
                self.robomas.state_vel = float(data[1]) / 60.*pi*2. / 36.
                self.robomas.state_curr = float(data[2]) / 10000.

def main(args=None):
    rclpy.init(args=args)
    node = TempNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
