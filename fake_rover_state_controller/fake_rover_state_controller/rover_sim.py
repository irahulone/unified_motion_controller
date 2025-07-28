import rclpy
from rclpy.node import Node

import time
import math

from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose2D
from pioneer_interfaces.msg import PioneerInfo

from .my_ros_module import PubSubManager

UPDATE_RATE = 0.1
VEL_ALIVE = 10
MAX_TRANS = 1.0
MAX_ROTATE = 1.0


class JointStates(Node):

    def __init__(self):
        super().__init__('rover_sim')
        self.pubsub = PubSubManager(self)
        
        self.declare_parameters(
            namespace='',
            parameters=[
                ('robot_id', "p0"),
                ('x', 0.0),
                ('y', 0.0),
                ('t', 0.0)
            ]
        )
        params = self._parameters
        for name, param in params.items():
            self.get_logger().info(f"PARAMS/ {name}: {param.value}")

        self.robot_id = self.get_parameter('robot_id').value
        
        x = self.get_parameter('x').value
        y = self.get_parameter('y').value
        t = self.get_parameter('t').value

        self.position = {'x': x, 'y': y, 'theta': t}
        self.position_hw = {'x': 0.0, 'y': 0.0, 'theta': 0.0}
        self.position_desired = {'x': 0.0, 'y': 0.0, 'theta': 0.0}
        self.vel = {'transform': 0.0, 'rotate': 0.0, 'alive': 0}
        self.joint_names = ['w_to_x', 'x_to_y', 'y_to_t']

        self.pubsub.create_subscription(Twist, f'/sim/{self.robot_id}/cmd_vel', self.teleop_callback, 10)
        self.pubsub.create_publisher(JointState, f'/{self.robot_id}/joint_states', 10)
        self.pubsub.create_publisher(Pose2D, f'/sim/{self.robot_id}/pose2D', 10)

        self.pubsub.create_subscription(Pose2D, f'/{self.robot_id}/pose2D', self.ghost_callback, 10)
        self.pubsub.create_publisher(JointState, f'/{self.robot_id}hw/joint_states', 10)

        self.pubsub.create_subscription(Pose2D, f'/{self.robot_id}/desiredPose2D', self.desired_callback, 10)
        self.pubsub.create_publisher(JointState, f'/{self.robot_id}desired/joint_states', 10)
        
        timer_period = UPDATE_RATE
        self.timer = self.create_timer(timer_period, self.publish_joint_states)

    def update_position(self):
        if self.vel['alive'] <= 0:
            return
        self.position['theta'] += UPDATE_RATE  * self.vel['rotate']
        self.position['theta'] = self.wrap_to_pi(self.position['theta'])
        self.position['x'] += UPDATE_RATE  * self.vel['transform'] * math.cos(self.position['theta'])
        self.position['y'] += UPDATE_RATE  * self.vel['transform'] * math.sin(self.position['theta'])
        self.vel['alive'] -= 1
    
    def wrap_to_pi(self, t):
        return (t + math.pi) % (2 * math.pi) - math.pi
        
    def publish_joint_states(self):
        jointstates_msg = JointState()
        t = time.time()
        jointstates_msg.header.stamp.sec = int(t // 1)
        jointstates_msg.header.stamp.nanosec = int((t % 1) * 1e9)
        self.update_position()
        jointstates_msg.name = [f'{joint_name}' for joint_name in self.joint_names]
        jointstates_msg.position = [
            self.position['x'], self.position['y'], self.position['theta']
        ]
        self.pubsub.publish(f'/{self.robot_id}/joint_states', jointstates_msg)
        pose_msg = Pose2D()
        pose_msg.x = self.position['x']
        pose_msg.y = self.position['y']
        pose_msg.theta = self.position['theta']
        self.pubsub.publish(f'/sim/{self.robot_id}/pose2D', pose_msg)

        jointstates_msg.name = [f'{joint_name}' for joint_name in self.joint_names]
        jointstates_msg.position = [
            self.position_hw['x'], self.position_hw['y'], self.position_hw['theta']
        ]
        self.pubsub.publish(f'/{self.robot_id}hw/joint_states', jointstates_msg)

        jointstates_msg.name = [f'{joint_name}' for joint_name in self.joint_names]
        jointstates_msg.position = [
            self.position_desired['x'], self.position_desired['y'], self.position_desired['theta']
        ]
        self.pubsub.publish(f'/{self.robot_id}desired/joint_states', jointstates_msg)

    def teleop_callback(self, msg):
        self.vel.update({'transform': max(-MAX_TRANS, min(msg.linear.x, MAX_TRANS)), 'rotate': max(-MAX_ROTATE, min(msg.angular.z, MAX_ROTATE)), 'alive': VEL_ALIVE})

    def ghost_callback(self, msg):
        self.position_hw.update({'x': msg.x, 'y': msg.y, 'theta': msg.theta})

    def desired_callback(self, msg):
        self.position_desired.update({'x': msg.x, 'y': msg.y, 'theta': msg.theta})


def main(args=None):
    rclpy.init(args=args)

    teleop_sub = JointStates()

    rclpy.spin(teleop_sub)

    teleop_sub.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()