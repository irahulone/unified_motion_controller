import rclpy
from rclpy.node import Node
from std_msgs.msg import Int16
from std_srvs.srv import SetBool
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool
from std_msgs.msg import String
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import NavSatFix

from .my_ros_module import PubSubManager, NavNode


INITIAL_DESIRED_COORDINATES = [37.35228, -121.941788] # Garage
#ROVER_FORMATION = [[5.0, 0.0, 0.0], [5.0, 120.0, 0.0], [5.0, 240.0, 0.0]] # [distance, angle, heading]
ROVER_FORMATION = [5.0, 0.0, 0.0,  5.0, 120.0, 0.0,  5.0, 240.0, 0.0] # [distance, angle, heading]

class NavController(NavNode):
    def __init__(self, node_name='nav_controller', target_lat_lon=None):
        super().__init__(node_name=node_name)

        self.declare_parameters(
            namespace='',
            parameters=[
                ('robot_id_list', ["p0"]),
                ('rover_formation', ROVER_FORMATION),
                ('scaling_coor', 0.05),
                ('rotating_coor', 1.0),
                ('scaling_max', 10.0),
                ('scaling_min', 2.0)
            ]
        )

        rover_formation = self.get_parameter('rover_formation').value
        self.rover_formation = [list(rover_formation[i:i+3]) for i in range(0, len(rover_formation), 3)]
        self.n_rover = len(self.rover_formation)

        self.scaling_coor = self.get_parameter('scaling_coor').value
        self.rotating_coor = self.get_parameter('rotating_coor').value
        self.scaling_max = self.get_parameter('scaling_max').value
        self.scaling_min = self.get_parameter('scaling_min').value

        self.cur_lat = []
        self.cur_lon = []
        self.cur_heading = []
        self.status_gps = [0]*self.n_rover
        robot_id_list = self.get_parameter('robot_id_list').value
        #self.get_logger().info(f"robot_id_list: {robot_id_list}")
        if len(robot_id_list) == 1 and robot_id_list[0] == "p0":
            robot_id_list = None
        self.robot_id_list = []
        if robot_id_list is None:
            for i in range(self.n_rover):
                self.robot_id_list.append(f"{self.robot_id_list[i]}")
        elif isinstance(robot_id_list, int):
            for i in range(self.n_rover):
                self.robot_id_list.append(f"p{i+1+robot_id_list}")
        elif isinstance(robot_id_list, list):
            if isinstance(robot_id_list[0], int):
                for i in range(self.n_rover):
                    self.robot_id_list.append(f"p{robot_id_list[i]}")
            else:
                self.robot_id_list = robot_id_list
        else:
            print("ROBOT_ID_LIST ERROR")
            return
        print(self.robot_id_list)
        if target_lat_lon is not None:
            self.des_lat, self.des_lon = target_lat_lon
        else:
            self.des_lat, self.des_lon = None, None
        self.status_des_gps = 0
        self.frame_des_heading = None

        for i in range(self.n_rover):
            self.cur_lat.append(None)
            self.cur_lon.append(None)
            self.cur_heading.append(None)
            self.pubsub.create_subscription(
                Float32MultiArray,
                f'/{self.robot_id_list[i]}/imu/eulerAngle',
                lambda msg, i=i: self.euler_callback(msg, i),
                5)
            self.pubsub.create_subscription(
                NavSatFix,
                f'/{self.robot_id_list[i]}/gps1',
                lambda msg, i=i: self.current_gps_callback(msg, i),
                5)
        self.pubsub.create_subscription(
            NavSatFix,
            '/nav/frame_target_gps',
            self.destination_gps_callback,
            5)
        self.pubsub.create_subscription(
            Float32MultiArray,
            '/nav/rover_formations',
            self.formation_callback,
            5)
        self.pubsub.create_subscription(
            Float32MultiArray,
            '/joy/cross',
            self.joy_formation_callback,
            5)
        self.pubsub.create_subscription(
            Float32MultiArray,
            '/nav/rover_formation_params',
            self.formation_params_callback,
            5)
        for i in range(self.n_rover):
            self.pubsub.create_publisher(
                NavSatFix,
                f'/nav/{self.robot_id_list[i]}/target_gps',

                5)
        # for test
        self.pubsub.create_publisher(
            Float32MultiArray,
            '/nav/current_des_formations',
            5)

        
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.nav_callback)

    def current_gps_callback(self, msg_cur_pos:NavSatFix, i):
        self.status_gps[i] = msg_cur_pos.status.status
        print("[Current] Status GPS:", self.status_gps , "Lat/Lon:", msg_cur_pos.latitude,msg_cur_pos.longitude)
        if self.status_gps[i] != 0:
            self.cur_lat[i] = msg_cur_pos.latitude
            self.cur_lon[i] = msg_cur_pos.longitude

    def destination_gps_callback(self, msg_des_pos:NavSatFix):
        self.status_des_gps = msg_des_pos.status.status
        print("[Destination] Status GPS:", self.status_gps , "Lat/Lon:", msg_des_pos.latitude,msg_des_pos.longitude)
        if self.status_des_gps != 0:
            self.des_lat = msg_des_pos.latitude
            self.des_lon = msg_des_pos.longitude
            
    def euler_callback(self, msg_imu_euler:Float32MultiArray, i):
        self.cur_heading[i] =  msg_imu_euler.data[0]
        print("[Current] Status IMU:", self.robot_id_list[i] , "Heading Angle:", self.cur_heading[i])

    def formation_callback(self, msg_formation:Float32MultiArray):
        if len(msg_formation.data) != self.n_rover*3:
            print("Invalid formation data : Number of rovers does not match. Current number of rovers: ", self.n_rover)
            return
        for i in range(self.n_rover):
            self.rover_formation[i] = msg_formation.data[i*3:i*3+3]
    
    def formation_params_callback(self, msg_formation_params:Float32MultiArray):
        distance, heading_offset = msg_formation_params.data
        for i in range(self.n_rover):
            self.rover_formation[i] = [distance, (i*360/self.n_rover+heading_offset)%360, 0.0]
    
    def joy_formation_callback(self, msg_joy_params:Float32MultiArray):
        scaling, rotating = msg_joy_params.data
        for i in range(self.n_rover):
            prev_distance, prev_angle, _ = self.rover_formation[i]
            self.rover_formation[i] = [min(max(self.scaling_min, prev_distance+scaling*self.scaling_coor),self.scaling_max), (prev_angle+rotating*self.rotating_coor)%360, 0.0]
    
    def nav_callback(self):
        print(f"Frame destination: [{self.des_lat}, {self.des_lon}, {self.frame_des_heading}, rover formation: {self.rover_formation}")
        msg_rover_pos = NavSatFix()
        for i in range(self.n_rover):
            msg_rover_pos.latitude, msg_rover_pos.longitude = self.get_target_pos(self.des_lat, self.des_lon, self.rover_formation[i][0], self.rover_formation[i][1])
            self.pubsub.publish(f'/nav/{self.robot_id_list[i]}/target_gps', msg_rover_pos)
            print(f"Rover {i+1} ({self.robot_id_list[i]}) target position: [{msg_rover_pos.latitude}, {msg_rover_pos.longitude}]")
        
        # for test
        msg_current_des_formations = Float32MultiArray()
        msg_current_des_formations.data = []
        for i in range(self.n_rover):
            msg_current_des_formations.data.extend(self.rover_formation[i])
        self.pubsub.publish('/nav/current_des_formations', msg_current_des_formations)


def main(args=None):
    rclpy.init(args=args)
    sub_move_cmds = NavController(target_lat_lon=INITIAL_DESIRED_COORDINATES)
    rclpy.spin(sub_move_cmds)
    sub_move_cmds.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()