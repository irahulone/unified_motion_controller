import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
from geometry_msgs.msg import Twist
#from rover_interfaces.msg import RoverCmds
from math import pi

import os


class GetMoveCmds(Node):

    def __init__(self):
        # Initialize the Node with the name 'movebase_kinematics'
        self.robot_id = os.getenv("ROBOT_ID")
        super().__init__(f'{self.robot_id}_movebase_kinematics')
        self.declare_parameter('max_vel', 50)
        self.max_vel = self.get_parameter('max_vel').value

        # Create a subscription to the 'cmd_vel' topic with a callback function 
        self.subscription = self.create_subscription(
            Twist,
            f'/{self.robot_id}/cmd_vel',
            self.move_cmd_callback, 
            5)
        self.subscription  

        # Create a publisher for the left & right wheel's control signal on the their topics
        self.pub_move = self.create_publisher(
            Int32MultiArray, 
            f'/{self.robot_id}/ch_vals',
            5)

        self.lx = 0.0
        self.az = 0.0
        self.vel_left = 0
        self.vel_right = 0

    def move_cmd_callback(self, msg):
        # Update 'linear_x' and 'angular_z' with the linear and angular commands from the message
        
        #if msg.id == 1:
        lx = msg.linear.x
        az = msg.angular.z

        # [EDIT THE TWO LINES BELOW] Calculate control signals based on 'linear_x' and 'angular_z' values 
        vel_left  =  -int(self.max_vel*(lx - az))
        vel_right = int(self.max_vel*(lx + az))

        # construct payload with left & rigth velocities 
        payload = Int32MultiArray()
        payload.data = [vel_left, vel_right]
    
        # Publish the message on the 'ch_vals' topic
        self.pub_move.publish(payload)
        #self.pub_ch1(40)

def main(args=None):
    # Initialize the ROS2 node
    rclpy.init(args=args)
    # Create an instance of the 'GetMoveCmds' class, which starts the subscription and timers
    sub_move_cmds = GetMoveCmds()
    # Enter the ROS2 event loop and wait for callbacks to be triggered
    rclpy.spin(sub_move_cmds)
    # Clean up and destroy the node when the event loop exits
    sub_move_cmds.destroy_node()
    # Shutdown the ROS2 client library
    rclpy.shutdown()

if __name__ == '__main__':
    main()
