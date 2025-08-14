import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose2D
from std_msgs.msg import Float32
from sensor_msgs.msg import NavSatFix
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
import csv
import os

import numpy
import math

class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('pose_listener')
        self.pose_sub = self.create_subscription(
            Pose2D,
            '/p2/pose2D',
            self.listener_callback,
            10)
        self.pose_sub  # prevent unused variable warning
      

        self.pub_cmdvel = self.create_publisher(Twist, '/p2/cmd_vel', 10)
        
        self.joy_sub = self.create_subscription(
            Joy,
            '/joy',
            self.joy_callback,
            10)
               
        self.joy_sub
        
        self.j_lx = 0.0
        self.j_az = 0.0 
        self.lx_axisN = 1
        self.az_axisN = 0
        self.en_buttonN = 4
        self.ulx=0.0
        self.uaz=0.0
        self.e_d=0.0
        self.e_h=0.0
        self.button = 0  


    def listener_callback(self, msg):
        self.e_d = self.j_lx #joy input translational
        self.e_h = self.j_az #joy input angular
        Kaz=0.5 #angular gain
        Klx=0.7 #translational gain
        Vx=0 #translational constant

        self.uaz = Kaz*self.e_h
        if self.uaz > 0.6:
            self.uaz = 0.6
        elif self.uaz < -0.6:
            self.uaz = -0.6
        
        self.ulx = Vx+Klx*self.e_d
        if self.ulx > 0.4:
            self.ulx = 0.4
        elif self.ulx < -0.4:
            self.ulx = -0.4
            

        msg_cmd = Twist()
        msg_cmd.linear.x = self.ulx
        msg_cmd.angular.z = self.uaz
        self.pub_cmdvel.publish(msg_cmd)
             
    def joy_callback(self, msg):
        self.j_lx = msg.axes[self.lx_axisN]
        self.j_az = msg.axes[self.az_axisN] 

def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
