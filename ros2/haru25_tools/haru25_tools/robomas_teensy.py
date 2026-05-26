# -*- coding: utf-8 -*-
import rclpy
from rclpy.node import Node
import sys
import serial
import struct
import time
import copy
from cobs import cobs # smart binary serial encoding and decoding

from std_msgs.msg import Float32MultiArray

serPort='/dev/ttyACM0'
ser = serial.Serial(serPort, baudrate=9600, bytesize=8, parity='N', stopbits=1, timeout=1)

zeroByte = b'\x00' # COBS 1-byte delimiter is hex zero as a (binary) bytes character

class TempNode(Node):#Nodeクラスを継承(nodeがつくれるようになる)

    def __init__(self):
        super().__init__('temp')
        self.robomas_pub_ = self.create_publisher(Float32MultiArray, 'robomas/rx', 10)
        self.robomas_sub_ = self.create_subscription(Float32MultiArray,'robomas/tx',self.robomas_cb, 10)
        
        print('opened serial port: {0}'.format(serPort))
        
        self.msg = None
        
        self.timer = self.create_timer(0.01, self.timer_cb)

    def timer_cb(self):
        try:
            if self.msg != None:
                data_num = len(self.msg.data)
                dataPacked = struct.pack("f"*data_num,*self.msg.data)
                dataEncoded = cobs.encode( dataPacked )
                nBytes = ser.write(dataEncoded + b'\x00' )
                strRead = ser.read_until( zeroByte )
                nBytesRead=len(strRead)
                if nBytesRead > 0:
                    decodeStr   = strRead[0:(nBytesRead-1)] 
                    dataDecoded = cobs.decode( decodeStr )
                    data_num = int((nBytesRead-2)/4)
                    s = struct.Struct('f' * data_num)
                    dataUnpacked = s.unpack(dataDecoded)
                    msg = Float32MultiArray()
                    msg.data = dataUnpacked
                    self.robomas_pub_.publish(msg)
        except Exception as e:
            print(e)
    
    def robomas_cb(self, msg):
        self.msg = copy.copy(msg)

def main(args=None):
    rclpy.init(args=args)
    node = TempNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
