import rclpy
from rclpy.node import Node

from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import Quaternion, Pose2D
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Int16MultiArray
from std_msgs.msg import Int16

import math

from pioneer_interfaces.srv import RefGPS

import datetime
import os
from math import sin, cos, asin, atan2, sqrt, degrees, pi, radians


class PoseConverter(Node):
    def __init__(self, n_rover=6):
        self.robot_id = os.getenv("ROBOT_ID")
        super().__init__(f'{self.robot_id}_pose_converter')
        # Temporary values / Garage
        #self.ref_lat = 37.35232
        #self.ref_lon = -121.94158
        self.ref_lat = None
        self.ref_lon = None
        self.lat = None
        self.lon = None
        self.gps_status = None
        self.quaternion = None
        self.euler_x = None
        self.calibration = None


        self.create_subscription(
            NavSatFix,
            f"/{self.robot_id}/gps1",
            self.gps_callback,
            1)
        self.create_subscription(
            Quaternion,
            f"/{self.robot_id}/imu/quaternion",
            self.quaternion_callback,
            1)
        self.create_subscription(
            Float32MultiArray,
            f"/{self.robot_id}/imu/eulerAngle",
            self.euler_callback,
            1)
        self.pose_publisher = self.create_publisher(
            Pose2D,
            f'/{self.robot_id}/pose2D',
            5)
        
        self.cli = self.create_client(RefGPS, 'reference_gps')

        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting...')
        
        self.req = RefGPS.Request()
        self.req.robot_id = self.robot_id
        self.request_reference_gps()

        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
    
    def request_reference_gps(self):
        self.future = self.cli.call_async(self.req)
        self.future.add_done_callback(self.srv_callback)
        self.get_logger().info(f'[{self.req.robot_id}]Sent request for reference GPS')
    
    def srv_callback(self, future):
        self.ref_lat = future.result().gps.latitude
        self.ref_lon = future.result().gps.longitude
        self.get_logger().info('Reference GPS received:')
        
    def gps_callback(self, msg):
        self.lat = msg.latitude
        self.lon = msg.longitude
        self.gps_status = msg.status.status
    
    def quaternion_callback(self, msg):
        self.quaternion = msg
    
    def euler_callback(self, msg):
        self.euler_x = msg.data[0]
        self.euler_y = msg.data[1]
        self.euler_z = msg.data[2]
        if self.calibration is None:
            self.calibration = self.euler_x
    
    def timer_callback(self):
        if self.ref_lat is None or self.ref_lon is None:
            self.get_logger().info("Reference GPS not set")
            self.request_reference_gps()
            return
        if self.lat is None or self.lon is None or self.quaternion is None or self.euler_x is None or self.calibration is None:
            self.get_logger().info("Not all data available")
            return
        msg = Pose2D()
        msg.x, msg.y = self.convert_gps_to_pose(self.lat, self.lon, self.ref_lat, self.ref_lon)
        msg.theta = self.degree_to_radian_pi_range(self.euler_x - self.calibration)
        self.pose_publisher.publish(msg)
        self.get_logger().info(f"Published: {msg.x}, {msg.y}, {msg.theta}")
        
    def degree_to_radian_pi_range(self, degree):
        # Convert degree to radian
        rad = math.radians(degree)
        # Normalize to [-π, π)
        rad = (rad + math.pi) % (2 * math.pi) - math.pi
        return rad
    
    def convert_gps_to_pose(self, cur_lat, cur_lon, ref_lat, ref_lon):
        R = 6371000
        
        delta_lat = radians(cur_lat - ref_lat)
        delta_lon = radians(cur_lon - ref_lon)
        
        ref_lat_rad = radians(ref_lat)
        
        x = R * delta_lon * cos(ref_lat_rad)
        y = R * delta_lat
        
        return x, y

def main(args=None):
    rclpy.init(args=args)
    converter = PoseConverter()
    rclpy.spin(converter)
    converter.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
