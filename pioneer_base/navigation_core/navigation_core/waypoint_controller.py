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

iniDesiredCoor = [37.35228, -121.941788] # Garage

class GiveDirections(Node):

    def __init__(self):

        # Publisher to give cmd_vel to movebase_kinematics (linear x, angular z)
        self.n_target_rover = 0
        self.target_rover = f'p{self.n_target_rover+1}'

        super().__init__(f'directions_publisher_for_{self.target_rover}')
        self.des_heading = 0
        self.dist = 0

        self.publisher_ = self.create_publisher(
        	Twist,
        	f'/{self.target_rover}/nav_cmd_vel', 
        	5)
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.give_dir)

        self.publisherImpData_ = self.create_publisher(
        	Float32MultiArray,
        	f'/{self.target_rover}/impData', 
        	5)
        timer_period = 0.25  # seconds
        self.timer = self.create_timer(timer_period, self.publish_data)

        # Create a subscription to get current Euler Angles from IMU
        self.cur_heading = 0 #Euler Angle (heading)
        self.subscriptionEuler = self.create_subscription(
            Float32MultiArray,
            f'/{self.target_rover}/imu/eulerAngle',
            self.euler_callback, 
            5)
        self.subscriptionEuler 

        # Create a subscription to get current location from GPS
        self.cur_lat = 0
        self.cur_lon = 0
        self.status_gps=True
        self.subscriptionLoc = self.create_subscription(
            NavSatFix,
            f'/{self.target_rover}/gps1',
            self.current_gps_callback,
            5)
        self.subscriptionLoc

        # # Create a subscription to get desired GPS location
        self.des_lat, self.des_lon  = iniDesiredCoor
        self.subscriptionTarget = self.create_subscription(
            NavSatFix,
            f'/{self.target_rover}/target',
            self.target_callback,
            5)
        self.subscriptionTarget

    def current_gps_callback(self, msg_cur_pos:NavSatFix):
        self.status_gps = msg_cur_pos.status.status
        print("Status GPS:", self.status_gps , "Lat/Lon:", msg_cur_pos.latitude,msg_cur_pos.longitude)
        if(self.status_gps!=0):
            self.cur_lat = msg_cur_pos.latitude
            self.cur_lon = msg_cur_pos.longitude

    def target_callback(self, msg_des_pos:NavSatFix):

        print("Desired Lat/Lon:", msg_des_pos.latitude,msg_des_pos.longitude)
        self.des_lat = msg_des_pos.latitude
        self.des_lon = msg_des_pos.longitude

    def euler_callback(self, msg_imu_euler:Float32MultiArray):
        if self.n_target_rover == 0:
            self.cur_heading = 360 - msg_imu_euler.data[0]
        elif self.n_target_rover == 1:
            self.cur_heading = msg_imu_euler.data[0]
        elif self.n_target_rover == 2:
            self.cur_heading = 360 - msg_imu_euler.data[0]
        
    def publish_data(self):
        data_msg = Float32MultiArray()
        data_msg.data = [float(self.cur_heading), float(self.des_heading), float(self.dist)]
        self.publisherImpData_.publish(data_msg)

    def get_bearing_distance(self, lat0, lon0, lat1, lon1):
        delta_lat = radians(lat1) - radians(lat0)
        delta_lon = radians(lon1) - radians(lon0)
        cos_lat1 = cos(radians(lat1))
        cos_lat0 = cos(radians(lat0))
        bearingX = cos_lat1 * sin(delta_lon)
        bearingY = cos_lat0 * sin(radians(lat1)) - sin(radians(lat0)) * cos_lat1 * cos(delta_lon)
        yaw = -atan2(bearingX,bearingY)
        if yaw<0:
            yaw += 2*pi
        bearing = degrees(yaw)

        R = 6373.0
        a = sin(delta_lat/2)**2 + cos_lat1 * cos_lat0 * sin(delta_lon/2)**2
        c = 2* atan2(sqrt(a), sqrt(1-a))
        dist = R*c*1000

        return bearing, dist

    def give_dir(self):
        delta_lat = radians(self.des_lat) - radians(self.cur_lat)
        delta_lon = radians(self.des_lon) - radians(self.cur_lon)
        cos_des_lat = cos(radians(self.des_lat))
        cos_cur_lat = cos(radians(self.cur_lat))
        bearingX = cos_des_lat * sin(delta_lon)
        bearingY = cos_cur_lat * sin(radians(self.des_lat)) - sin(radians(self.cur_lat)) * cos_des_lat * cos(delta_lon)
        yaw_target = -atan2(bearingX,bearingY)
        # yaw_target_approx = -atan2(delta_lon,delta_lat)
        if yaw_target<0:
            yaw_target += 2*pi
        # yaw_target = 90
        yaw_delta = degrees(yaw_target) - self.cur_heading
        self.des_heading = degrees(yaw_target)

        R = 6373.0
        a = sin(delta_lat/2)**2 + cos_des_lat * cos_cur_lat * sin(delta_lon/2)**2
        c = 2* atan2(sqrt(a), sqrt(1-a))
        dist = R*c*1000

        self.dist=dist

        # for 3 rovers
        self.coefficient = [[1/5, 1.2, 1.0, 1.2, 2.0], [-1/5, -1.2, -1.0, -1.2, -2.0], [1/5, 1.2, 1.0, 1.8, 2.4]]
        
        msg = Twist()
        if dist < 2.5:
            msg.linear.x = 0
            msg.angular.z = 0
            print("STOPPED, ", dist, degrees(yaw_target), self.cur_heading, yaw_delta)
        elif dist < 5:
            msg.linear.x = self.coefficient[self.n_target_rover][0] * dist
            msg.angular.z = self.coefficient[self.n_target_rover][1] * yaw_delta/360
            print("PARKING, ", dist, degrees(yaw_target), self.cur_heading, yaw_delta)
        elif yaw_delta < 90 and yaw_delta > -90:
            msg.linear.x = self.coefficient[self.n_target_rover][2]
            msg.angular.z = self.coefficient[self.n_target_rover][3] * yaw_delta/360
            print("TRAVELLING, ", dist, degrees(yaw_target), self.cur_heading, yaw_delta)
        else:
            msg.linear.x = 0.0
            msg.angular.z = self.coefficient[self.n_target_rover][4] * yaw_delta/360
            print("TURNING, ", dist, degrees(yaw_target), self.cur_heading, yaw_delta)

        #print(msg)
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)

    Directions = GiveDirections()
    rclpy.spin(Directions)
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    Directions.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
