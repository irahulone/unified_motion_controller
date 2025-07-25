from rclpy.node import Node
from std_msgs.msg import Int32MultiArray, Int64MultiArray
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
import time

# Custom submodules
from roboteq_core.status import Status

import logging
logging.basicConfig(level=logging.WARNING)

QUEUESIZE = 10


class Publisher(Node):
    """
    @class: Publish any stats regarding the drive inverter
    """
    def __init__(self, connection, encoder_topic, velocity_topic):
        super().__init__('roboteq_drive_inverter_pub')
        self.get_stat = Status(connection)

        # Init publishers
        self.du_velocity_period = 0.1  # seconds
        # self.du_encoder_callback_period = 0.1 # seconds
        self.publish_velocity = self.create_publisher(Int32MultiArray, f"{velocity_topic}", 10)
        self.publish_encoder_val = self.create_publisher(Int64MultiArray, f"{encoder_topic}", 10)
        
        self.create_timer(self.du_velocity_period,
                          self.du_encoder_callback,
                          callback_group=MutuallyExclusiveCallbackGroup())
        
        self.create_timer(self.du_velocity_period,
                          self.du_velocity_callback,
                          callback_group=MutuallyExclusiveCallbackGroup())

        # Init subscribers
        self.create_subscription(
            Int64MultiArray,
            f"{encoder_topic}",
            self.calculate_velocity_callback,
            QUEUESIZE,
            callback_group=MutuallyExclusiveCallbackGroup()
        )
        # self.velocity_seconds = 1
        self.velocities = [(0, 0)]

        self.i = 0


    def calculate_velocity_callback(self, msg):
        time.sleep(self.du_velocity_period)
        current = self.get_stat.read_encoder()
        velocity_left = ((current[0] - msg.data[0]) / self.du_velocity_period) * (6 / 10000) # rpm
        velocity_right = ((current[1] - msg.data[1]) / self.du_velocity_period) * (6 / 10000) # rpm
        self.velocities.append((velocity_left, velocity_right))


    def du_encoder_callback(self):
        """
        Get the encoder values of the motor
        """
        msg = Int64MultiArray()
        msg.data = [value for value in self.get_stat.read_encoder()]
        self.publish_encoder_val.publish(msg)
        # self.get_logger().info(f'Publishing: {msg.data}')

        self.i += 1

    def du_velocity_callback(self):
        msg = Int32MultiArray()

        # current contains a list of tuples for all the calculated velocities (right, left) from the subscriber calculate_velocity_callback
        current = [(reading[0], reading[1]) for reading in self.velocities if reading]
        if len(current):
            msg.data = [int(current[0][0]), int(current[0][1])]
            self.velocities.pop(0)
            self.publish_velocity.publish(msg)
            # self.get_logger().info(f'Publishing: {msg.data}')
        


        self.i += 1
