import rclpy
from rclpy.node import Node

from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float32

from ublox_gps import UbloxGps
import serial
import numpy as np
import math

DEBUG = false

timer_period = 0.1  # seconds

port_gps1 = serial.Serial('/dev/ttyACM1', baudrate=38400, timeout=1)
port_gps2 = serial.Serial('/dev/ttyACM0', baudrate=38400, timeout=1)

gps1 = UbloxGps(port_gps1)
gps2 = UbloxGps(port_gps2)

class read_gps_data(Node):

    def __init__(self):
        super().__init__('pub')
        self.publisher_gps1 = self.create_publisher(NavSatFix, '/r1/gps1', 2)
        self.publisher_gps2 = self.create_publisher(NavSatFix, '/r1/gps2', 2)
        self.publisher_gps_agg = self.create_publisher(NavSatFix, '/r1/gps_agg', 2)
        self.publisher_brng = self.create_publisher(Float32, '/r1/heading', 2)
        self.timer = self.create_timer(timer_period, self.gps_callback)

    def get_bearing(self, lat1, long1, lat2, long2):
        dLon_rad = math.radians(long2 - long1)
        #print(long1)

        lat1_rad = math.radians(lat2)
        lat2_rad = math.radians(lat1)
        long1_rad = math.radians(long1)
        long2_rad = math.radians(long2)

        x = math.cos(lat2_rad) * math.sin(dLon_rad)
        y = math.cos(lat1_rad) * math.sin(lat2_rad) - math.sin(lat1_rad) * math.cos(lat2_rad) * math.cos(dLon_rad)
        
        brng = np.arctan2(x,y)
        brng = 90 -  math.degrees(brng) - 30

        if (brng < 0):
            brng += 360
        print(brng)
        brng_msg = Float32()
        brng_msg.data = brng
        self.publisher_brng.publish(brng_msg)

    def aggregate_gps(self, lat1, long1, lat2, long2):
        
        msg_agg = NavSatFix()
        msg_agg.latitude = (lat1+lat2)/2
        msg_agg.longitude = (long1+long2)/2
        
        self.publisher_gps_agg.publish(msg_agg)
        

    def gps_callback(self):
        gps1_lat = 0.0
        gps1_lon = 0.0
        gps2_lat = 0.0
        gps2_lon = 0.0
        try:
            coords_gps1 = gps1.geo_coords()
            coords_gps2 = gps2.geo_coords()
            
        except (ValueError, IOError) as err:
            if DEBUG:
                print("gps failure")
            return

        gps1_lat = coords_gps1.lat
        gps1_lon = coords_gps1.lon
        gps2_lat = coords_gps2.lat
        gps2_lon = coords_gps2.lon

        if DEBUG:
            print(coorids.lon, coords.lat)

        self.get_bearing(gps1_lat, gps1_lon, gps2_lat, gps2_lon)
        self.aggregate_gps(gps1_lat, gps1_lon, gps2_lat, gps2_lon)

        msg1 = NavSatFix()
        msg1.latitude = gps1_lat
        msg1.longitude = gps1_lon
        
        msg2 = NavSatFix()
        msg2.latitude = gps2_lat
        msg2.longitude = gps2_lon

        self.publisher_gps1.publish(msg1)
        self.publisher_gps2.publish(msg2)


def main(args=None):
    rclpy.init(args=args)

    pub = read_gps_data()

    rclpy.spin(pub)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
