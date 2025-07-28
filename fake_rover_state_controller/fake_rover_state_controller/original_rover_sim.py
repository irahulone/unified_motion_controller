import rclpy
from rclpy.node import Node

import time
import math

from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose2D
from pioneer_interfaces.msg import PioneerInfo

from .my_ros_module import PubSubManager

UPDATE_RATE = 0.01
TRANSFORM_VELOCITY_METER_PER_SEC = 0.5
ANGLE_VELOCITY_RADIAN_PER_SEC = 0.4
VEL_ALIVE = 5


class JointStates(Node):

    def __init__(self):
        super().__init__('rover_sim')
        self.pubsub = PubSubManager(self)

        self.pubsub.create_subscription(PioneerInfo, 'registered_robot', self.robot_register_callback)
        self.pubsub.create_subscription(PioneerInfo, 'removed_robot', self.robot_remove_callback)

        self.positions = {}
        self.positions_hw = {}
        self.vels = {}
        self.joint_names = ['w_to_x', 'x_to_y', 'y_to_t']
        
        timer_period = UPDATE_RATE
        self.timer = self.create_timer(timer_period, self.publish_joint_states)

    def update_position(self, robot_name, lx, az):
        key = robot_name
        if self.vels[key]['alive'] <= 0:
            return
        self.positions[key]['theta'] += UPDATE_RATE * ANGLE_VELOCITY_RADIAN_PER_SEC * az
        self.positions[key]['x'] += UPDATE_RATE * TRANSFORM_VELOCITY_METER_PER_SEC * lx * math.cos(self.positions[key]['theta'])
        self.positions[key]['y'] += UPDATE_RATE * TRANSFORM_VELOCITY_METER_PER_SEC * lx * math.sin(self.positions[key]['theta'])
        self.vels[key]['alive'] -= 1

    def update_vel(self, robot_name, lx, az):
        key = robot_name
        if key not in self.vels.keys():
            return
        self.vels[key]['transform'] = lx
        self.vels[key]['rotate'] = az
        self.vels[key]['alive'] = VEL_ALIVE
        
    def publish_joint_states(self):
        jointstates_msg = JointState()
        t = time.time()
        jointstates_msg.header.stamp.sec = int(t // 1)
        jointstates_msg.header.stamp.nanosec = int((t % 1) * 1e9)
        for key in self.vels.keys():
            self.update_position(key, self.vels[key]['transform'], self.vels[key]['rotate'])
            jointstates_msg.name = [f'{joint_name}' for joint_name in self.joint_names]
            jointstates_msg.position = [
                self.positions[key]['x'], self.positions[key]['y'], self.positions[key]['theta']
            ]
            #self.pubsub.publish(f'joint_states', jointstates_msg)
            self.pubsub.publish(f'/{key}/joint_states', jointstates_msg)
            pose_msg = Pose2D()
            pose_msg.x = self.positions[key]['x']
            pose_msg.y = self.positions[key]['y']
            pose_msg.theta = self.positions[key]['theta']
            self.pubsub.publish(f'/sim/{key}/pose2D', pose_msg)

            jointstates_msg.name = [f'{joint_name}' for joint_name in self.joint_names]
            jointstates_msg.position = [
                self.positions_hw[key]['x'], self.positions_hw[key]['y'], self.positions_hw[key]['theta']
            ]
            self.pubsub.publish(f'/{key}hw/joint_states', jointstates_msg)

    def teleop_callback(self, msg, robot_name):
        self.update_vel(robot_name, msg.linear.x, msg.angular.z)

    def ghost_callback(self, msg, robot_name):
        self.positions_hw[robot_name]['x'] = msg.x
        self.positions_hw[robot_name]['y'] = msg.y
        self.positions_hw[robot_name]['theta'] = msg.theta
    
    def robot_register_callback(self, msg):
        self.positions.update({msg.robot_id: {'x': msg.pose.x, 'y': msg.pose.y, 'theta': msg.pose.theta}})
        self.vels.update({msg.robot_id: {'transform': 0.0, 'rotate': 0.0, 'alive': 0}})
        self.get_logger().info(f'Received request: Register / {msg.robot_id}')
        self.pubsub.create_subscription(Twist, f'/sim/{msg.robot_id}/cmd_vel', lambda msg, name=msg.robot_id: self.teleop_callback(msg, name), 10)
        self.pubsub.create_publisher(JointState, f'/{msg.robot_id}/joint_states', 10)
        self.pubsub.create_publisher(Pose2D, f'/sim/{msg.robot_id}/pose2D', 10)

        self.positions_hw.update({msg.robot_id: {'x': msg.pose.x, 'y': msg.pose.y, 'theta': msg.pose.theta}})
        self.pubsub.create_subscription(Pose2D, f'/{msg.robot_id}/pose2D', lambda msg, name=msg.robot_id: self.ghost_callback(msg, name), 10)
        self.pubsub.create_publisher(JointState, f'/{msg.robot_id}hw/joint_states', 10)
    
    def robot_remove_callback(self, msg):
        self.positions.pop(msg.robot_id)
        self.vels.pop(msg.robot_id)
        self.get_logger().info(f'Received request: Remove / {msg.robot_id}')


def main(args=None):
    rclpy.init(args=args)

    teleop_sub = JointStates()

    rclpy.spin(teleop_sub)

    teleop_sub.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()