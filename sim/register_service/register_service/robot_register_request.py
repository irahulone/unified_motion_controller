import rclpy
from rclpy.node import Node
from pioneer_interfaces.srv import RegRobot
import sys

class RobotRegisterRequest(Node):
    def __init__(self):
        super().__init__('robot_register_req')
        self.declare_parameters(
            namespace='',
            parameters=[
                ('robot_id', "p1"),
                ('x', 0.0),
                ('y', 0.0),
                ('theta', 0.0),
            ]
        )
        self.cli = self.create_client(RegRobot, 'robot_register')

        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting...')
        
        self.req = RegRobot.Request()
        self.req.reg = True
        self.req.robot_id = self.get_parameter('robot_id').value
        self.req.pose.x = self.get_parameter('x').value
        self.req.pose.y = self.get_parameter('y').value
        self.req.pose.theta = self.get_parameter('theta').value
        self.future = self.cli.call_async(self.req)
        self.future.add_done_callback(self.callback)
    
    def callback(self, future):
        try:
            response = future.result()
            if response.success:
                self.get_logger().info('Registered.')
                sys.exit(0)  # 正常終了
            else:
                self.get_logger().warn(f'Reject: {self.req.robot_id}')
                sys.exit(1)  # エラー終了
        except Exception as e:
            self.get_logger().error(f'Fail to connect to the register server: {e}')
            sys.exit(1)


def main():
    rclpy.init()
    node = RobotRegisterRequest()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
