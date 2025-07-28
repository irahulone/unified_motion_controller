 #!/usr/bin/env python
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from std_msgs.msg import Int16
from std_msgs.msg import Int16MultiArray
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import NavSatFix
from math import sin, cos, atan2, sqrt, degrees, pi, radians


from .my_ros_module import PubSubManager


class HeadingController(Node):

    def __init__(self):
        super().__init__('heading_controller')
        self.declare_parameters(
            namespace='',
            parameters=[
                ('rover_id', "p0")
            ]
        )
        self.target_rover = self.get_parameter('rover_id').value
        self.get_logger().info(f"Controller:{self.target_rover}")


        self.desired_heading = 0
        self.actual_heading = 0
        self.k = 0.05

        self.pubsub = PubSubManager(self)
        self.pubsub.create_subscription(
            Int16,
            f'/heading/{self.target_rover}/actual',
            self.heading_callback,
            5)
        self.pubsub.create_subscription(
            Int16,
            f'/heading/{self.target_rover}/desired',
            self.angle_pub_callback,
            5)
        self.pubsub.create_publisher(
            Twist,
            f'/heading/{self.target_rover}/anglar_vel',
            5)
        
    def heading_callback(self, msg):
        heading = msg.data
        self.get_logger().info(f"{self.target_rover}/ Received heading: {heading}")
        self.actual_heading = heading
    
    def angle_pub_callback(self, msg):
        angle = msg.data
        self.get_logger().info(f"{self.target_rover}/ Received desired heading: {angle}")
        self.desired_heading = angle
        if self.actual_heading is None:
            return
        
        e = (self.desired_heading - self.actual_heading +180) % 360 - 180
        vel = Twist()
        vel.angular.z = self.k * e
        vel.linear.x = 0.0
        self.pubsub.publish(f'/heading/{self.target_rover}/anglar_vel', vel)
        self.get_logger().info(f"{self.target_rover}/ Send angular vel: {vel.angular.z}")


def main(args=None):
    rclpy.init(args=args)

    heading_controller = HeadingController()
    rclpy.spin(heading_controller)
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    heading_controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
