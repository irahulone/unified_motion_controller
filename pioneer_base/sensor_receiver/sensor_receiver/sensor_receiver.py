import rclpy
from rclpy.node import Node

from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import Quaternion
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Int16MultiArray
from std_msgs.msg import Int16

import datetime
import os

from .my_ros_module import PubSubManager


class SensorReceiver(Node):
    def __init__(self, n_rover=6):
        super().__init__(f'sensor_receiver')
        self.declare_parameters(
            namespace='',
            parameters=[
                ('robot_id_list', ["p0"])
            ]
        )
        params = self._parameters
        for name, param in params.items():
            self.get_logger().info(f"PARAMS/ {name}: {param.value}")
        
        self.timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        self.log_path = f"./pioneer_base/sensor_receiver/log/{self.timestamp}"
        os.makedirs(self.log_path, exist_ok=False)
        self.get_logger().info(f"Log dir: {self.log_path}")

        
        robot_id_list = self.get_parameter('robot_id_list').value
        self.get_logger().info(f"robot_id_list: {robot_id_list}")
        if len(robot_id_list) == 1 and robot_id_list[0] == "p0":
            robot_id_list = None
        self.robot_id_list = []
        if robot_id_list is None:
            self.n_rover = n_rover
            for i in range(self.n_rover):
                self.robot_id_list.append(f"{self.robot_id_list[i]}")
        elif isinstance(robot_id_list, int):
            self.n_rover = n_rover
            for i in range(self.n_rover):
                self.robot_id_list.append(f"p{i+1+robot_id_list}")
        elif isinstance(robot_id_list, list):
            self.n_rover = len(robot_id_list)
            if isinstance(robot_id_list[0], int):
                for i in range(self.n_rover):
                    self.robot_id_list.append(f"p{robot_id_list[i]}")
            else:
                self.robot_id_list = robot_id_list
        else:
            print("ROBOT_ID_LIST ERROR")
            return
        self.get_logger().info(f"ROVER: {self.robot_id_list} N: {self.n_rover}")

        self.pubsub = PubSubManager(self)
        for i in range(self.n_rover):
            self.get_logger().info(f"ROVER: {self.robot_id_list[i]}")
            self.pubsub.create_subscription(
                NavSatFix,
                f"/{self.robot_id_list[i]}/gps1",
                lambda msg, i=i: self.gps_callback(msg, i),
                1)
            self.pubsub.create_subscription(
                Quaternion,
                f"/{self.robot_id_list[i]}/imu/quaternion",
                lambda msg, i=i: self.quaternion_callback(msg, i),
                1)
            self.pubsub.create_subscription(
                Float32MultiArray,
                f"/{self.robot_id_list[i]}/imu/eulerAngle",
                lambda msg, i=i: self.euler_callback(msg, i),
                1)
            open(f"{self.log_path}/{self.robot_id_list[i]}_gps.txt", "w").close()
            open(f"{self.log_path}/{self.robot_id_list[i]}_quaternion.txt", "w").close()
            open(f"{self.log_path}/{self.robot_id_list[i]}_euler.txt", "w").close()
            self.pubsub.create_publisher(
                Int16,
                f'/heading/{robot_id_list[i]}/actual',
                5)
            self.get_logger().info(f"Set-up Complete:{self.robot_id_list[i]}")
        
    def gps_callback(self, msg, i):
        lat = msg.latitude
        lon = msg.longitude
        if lat is not None and lon is not None:
            with open(f"{self.log_path}/{self.robot_id_list[i]}_gps.txt", "a") as f:
                f.write(f"{datetime.datetime.now().strftime('%Y%m%d_%H%M%S')}, {lat}, {lon}\n")
        self.get_logger().info(f"{self.robot_id_list[i]}/gps: Lat:{lat}, Lon:{lon}")
    
    def quaternion_callback(self, msg, i):
        with open(f"{self.log_path}/{self.robot_id_list[i]}_quaternion.txt", "a") as f:
            f.write(f"{datetime.datetime.now().strftime('%Y%m%d_%H%M%S')}, {msg.x}, {msg.y}, {msg.z}, {msg.w}\n")
        self.get_logger().info(f"{self.robot_id_list[i]}/imu/quaternion: {msg.x}, {msg.y}, {msg.z}, {msg.w}")
    
    def euler_callback(self, msg, i):
        x = msg.data[0]
        y = msg.data[1]
        z = msg.data[2]
        with open(f"{self.log_path}/{self.robot_id_list[i]}_euler.txt", "a") as f:
            f.write(f"{datetime.datetime.now().strftime('%Y%m%d_%H%M%S')}, {x}, {y}, {z}\n")
        self.get_logger().info(f"{self.robot_id_list[i]}/imu/euler: {x}, {y}, {z}")
        pub_msg = Int16()
        pub_msg.data = int(x)
        self.pubsub.publish(f'/heading/{self.robot_id_list[i]}/actual',pub_msg)
        


def main(args=None):
    rclpy.init(args=args)
    sensor_receiver = SensorReceiver()
    rclpy.spin(sensor_receiver)
    sensor_receiver.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

#ros2 run joy joy_node
