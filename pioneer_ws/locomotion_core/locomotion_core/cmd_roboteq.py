import rclpy
from rclpy.node import Node
from std_msgs.msg import Int16
from std_msgs.msg import Int32MultiArray

import serial
import os

class MotorDriver(Node):
    def __init__(self, serial_port='/dev/ttyACM0', baudrate=115200):
        self.robot_id = os.getenv("ROBOT_ID")
        super().__init__(f'{self.robot_id}_cmd_roboteq')

        self.roboteq_obj = serial.Serial(
            port=serial_port,
            baudrate=baudrate,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            bytesize=serial.EIGHTBITS,
            timeout=1
            )

        self.subscription = self.create_subscription(
            Int32MultiArray,
            f'/{self.robot_id}/ch_vals',
            self.cmd_callback,
            5)
        self.subscription  # prevent unused variable warning

    def move_motor_ch1(self, val):
        payload1 = f"!G 1 {val}_"
        self.roboteq_obj.write(str.encode(payload1))
    
    def move_motor_ch2(self, val):
        payload2 = f"!G 2 {-val}_"
        self.roboteq_obj.write(str.encode(payload2))
            
    def cmd_callback(self, msg):
        inCmd1 = msg.data[0]
        inCmd2 = msg.data[1]
        self.move_motor_ch1(inCmd1)
        self.move_motor_ch2(inCmd2)


def main(args=None):
    rclpy.init(args=args)
    minimal_subscriber = MotorDriver()
    rclpy.spin(minimal_subscriber)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
