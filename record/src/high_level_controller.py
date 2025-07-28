import rclpy
from rclpy.node import Node

import time
import math
import numpy as np

from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose2D
from std_msgs.msg import Float32MultiArray
from pioneer_interfaces.msg import PioneerInfo

from .my_ros_module import PubSubManager

UPDATE_RATE = 0.1


class HighLevelClusterController(Node):

    def __init__(self, range=100000):
        super().__init__('high_level_cluster_controller')
        self.pubsub = PubSubManager(self)

        self.pubsub.create_publisher(Float32MultiArray, '/cluster_desired', 5)

        self.x = np.linspace(0.0, range, range*30)
        self.head = 0
        self.range = range

        self.publish_cluster_pose()

        timer_period = UPDATE_RATE
        self.timer = self.create_timer(timer_period, self.publish_cluster_pose)

    def publish_cluster_pose(self):
        msg = Float32MultiArray()
        _x = self.x[self.head] / 2
        msg.data = [_x, math.sin(_x/3) * 10, _x]
        self.head += 1
        self.pubsub.publish('/cluster_desired', msg)
        if self.head == self.range:
            self.destroy_node()


def main(args=None):
    rclpy.init(args=args)

    high_level_controller = HighLevelClusterController()
    rclpy.spin(high_level_controller)
    high_level_controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()