import rclpy
from rclpy.node import Node
from pioneer_interfaces.srv import RegRobot
import sys

class RobotRegisterCommand:
    def __init__(self, node: Node):
        self.node = node
        self.robots = {}
        self.cli = self.node.create_client(RegRobot, 'robot_register')

        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting...')
    
    def register(self, robot_id, x=0.0, y=0.0, theta=0.0):
        if robot_id in self.robots:
            self.node.get_logger().info(f'Robot {robot_id} is already registered.')
            return False
        req = RegRobot.Request()
        req.reg = True
        req.robot_id = self.get_parameter('robot_id').value
        req.pose.x = self.get_parameter('x').value
        req.pose.y = self.get_parameter('y').value
        req.pose.theta = self.get_parameter('theta').value
        future = self.cli.call_async(req)
        rclpy.spin_until_future_complete(self.node, future)
        if future.result().success:
            self.robots.update({robot_id: {'x': x, 'y': y, 'theta': theta}})
            return True
        else:
            return False
    
    def remove(self, robot_id):
        if robot_id not in self.robots:
            self.node.get_logger().info(f'Robot {robot_id} is not registered.')
            return False
        req = RegRobot.Request()
        req.reg = False
        req.robot_id = robot_id
        future = self.cli.call_async(req)
        rclpy.spin_until_future_complete(self.node, future)
        if future.result().success:
            self.robots.pop(robot_id)
            return True
        else:
            return False