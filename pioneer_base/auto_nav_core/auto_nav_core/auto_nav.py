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


iniDesiredCoor = [37.35228, -121.941788] # Garage

# Auto Navigation Node for single rover
class AutoNav(NavNode):
    def __init__(self, node_name='auto_nav', target_lat_lon=None, th_distance=0.5, th_heading=5):
        super().__init__(node_name=node_name)
        self.declare_parameter('robot_id', 'p1')
        self.target_rover = self.get_parameter('robot_id').value
        print(self.target_rover)
        self.th_distance = th_distance # meters
        self.th_heading = th_heading # degrees

        self.pubsub.create_subscription(
            Float32MultiArray,
            f'/{self.target_rover}/imu/eulerAngle',
            self.euler_callback, 
            5)
        self.pubsub.create_subscription(
            NavSatFix,
            f'/{self.target_rover}/gps1',
            self.current_gps_callback,
            5)
        self.pubsub.create_subscription(
            NavSatFix,
            f'/nav/{self.target_rover}/target_gps',
            self.destination_gps_callback,
            5)
        
        self.rover_status = 0
        self.cur_lat = None
        self.cur_lon = None
        if target_lat_lon is not None:
            self.des_lat, self.des_lon = target_lat_lon
        else:
            self.des_lat, self.des_lon = None, None
        self.status_gps = 0
        self.cur_heading = None

        self.pubsub.create_publisher(
        	Twist,
        	f'/nav/{self.target_rover}/cmd_vel', 
        	5)
        self.pubsub.create_publisher(
        	Int16,
        	f'/nav/{self.target_rover}/status', 
        	5)
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.nav_callback)
        
        msg_status = Int16()
        msg_status.data = 0
        self.pubsub.publish(f'/nav/{self.target_rover}/status',msg_status)

    def current_gps_callback(self, msg_cur_pos:NavSatFix):
        self.status_gps = msg_cur_pos.status.status
        #print("[Current] Status GPS:", self.status_gps , "Lat/Lon:", msg_cur_pos.latitude,msg_cur_pos.longitude)
        if self.status_gps != 0:
            self.cur_lat = msg_cur_pos.latitude
            self.cur_lon = msg_cur_pos.longitude

    def destination_gps_callback(self, msg_des_pos:NavSatFix):
        #print("[Destination] Lat/Lon:", msg_des_pos.latitude,msg_des_pos.longitude)
        self.des_lat = msg_des_pos.latitude
        self.des_lon = msg_des_pos.longitude
            
    def euler_callback(self, msg_imu_euler:Float32MultiArray):
        self.cur_heading =  msg_imu_euler.data[0]
        #print("[Current] Status IMU:", self.target_rover , "Heading Angle:", self.cur_heading)
    
    def nav_callback(self):
        if self.cur_lat is None or self.cur_lon is None or self.des_lat is None or self.des_lon is None or self.cur_heading is None:
            print(f"No GPS or Heading data: cur_lat={self.cur_lat}, cur_lon={self.cur_lon}, des_lat={self.des_lat}, des_lon={self.des_lon}, cur_heading={self.cur_heading}")
            return

        self.des_heading, self.des_distance = self.get_bearing_distance(self.cur_lat, self.cur_lon, self.des_lat, self.des_lon)
        print("[Destination] Heading/distance: ",self.des_heading, self.des_distance)

        e_h = self.des_heading - self.cur_heading 
        e_d = self.des_distance

        uaz = 0.06*e_h
        uaz = 0.6 if uaz > 0.6 else -0.6 if uaz < -0.6 else uaz
        
        ulx = 0.2*e_d
        ulx = 0.4 if ulx > 0.4 else -0.4 if ulx < -0.4 else ulx

        if e_h < self.th_heading and e_d < self.th_distance:
            msg_status = Int16()
            msg_status.data = 2
            self.pubsub.publish(f'/nav/{self.target_rover}/status',msg_status)
        else:
            msg_status = Int16()
            msg_status.data = 1
            self.pubsub.publish(f'/nav/{self.target_rover}/status',msg_status)

        msg_cmd = Twist()
        msg_cmd.linear.x = ulx
        msg_cmd.angular.z = uaz
        self.pubsub.publish(f'//nav/{self.target_rover}/cmd_vel',msg_cmd)


def main(args=None):
    rclpy.init(args=args)
    sub_move_cmds = AutoNav()
    rclpy.spin(sub_move_cmds)
    sub_move_cmds.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()