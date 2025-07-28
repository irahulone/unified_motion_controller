import rclpy
from rclpy.node import Node
from pioneer_interfaces.srv import RegRobot
import sys
import signal


class RobotRemoveRequest(Node):
    def __init__(self):
        super().__init__('robot_remove_req')
        self.declare_parameters(
            namespace='',
            parameters=[
                ('robot_id', "p1")
            ]
        )
        self.cli = self.create_client(RegRobot, 'robot_register')

        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting...')
        
        signal.signal(signal.SIGINT, self.shutdown_request)

    def shutdown_request(self, signum, frame):
        self.get_logger().info('Sending Remove request...')
        
        self.req = RegRobot.Request()
        self.req.reg = False
        self.req.robot_id = self.get_parameter('robot_id').value
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)

        if self.future.result() is not None:
            self.get_logger().info('Success removing robot.')
        else:
            self.get_logger().warn('Failed to remove robot.')

        rclpy.shutdown()
        sys.exit(0)

def main():
    rclpy.init()
    node = RobotRemoveRequest()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
