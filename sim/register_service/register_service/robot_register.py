import rclpy
from rclpy.node import Node
from pioneer_interfaces.srv import RegRobot
from pioneer_interfaces.msg import PioneerInfo

class RobotRegisterServer(Node):
    def __init__(self):
        super().__init__('robot_register')
        self.srv = self.create_service(RegRobot, 'robot_register', self.robot_register_callback)
        self.get_logger().info('Robot Register Server is ready.')

        self.robot = {}
        self.pub_reg = self.create_publisher(PioneerInfo, 'registered_robot', 10)
        self.pub_rm = self.create_publisher(PioneerInfo, 'removed_robot', 10)

    def robot_register_callback(self, request, response):        
        if request.reg:
            robot_id = request.robot_id
            if robot_id in self.robot:
                response.success = False
                self.get_logger().info(f'Received request: Register / {request.robot_id} -> {response.success}')
                return response
            
            self.robot.update({robot_id: {'x': request.pose.x, 'y': request.pose.y, 'theta': request.pose.theta}})
            response.success = True
            self.get_logger().info(f'Received request: Register / {request.robot_id} -> {response.success}')
            msg = PioneerInfo()
            msg.robot_id = robot_id
            msg.pose.x = request.pose.x
            msg.pose.y = request.pose.y
            msg.pose.theta = request.pose.theta
            self.pub_reg.publish(msg)
            return response
        else:
            if request.robot_id not in self.robot:
                response.success = False
                self.get_logger().info(f'Received request: Remove / {request.robot_id} -> {response.success}')
                return response
            
            self.robot.pop(request.robot_id)
            response.success = True
            self.get_logger().info(f'Received request: Remove / {request.robot_id} -> {response.success}')
            msg = PioneerInfo()
            msg.robot_id = request.robot_id
            self.pub_rm.publish(msg)
            return response

def main():
    rclpy.init()
    node = RobotRegisterServer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
