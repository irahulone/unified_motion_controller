import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix

import adafruit_gps
import serial
import os
import datetime

# Create a serial connection for the GPS connection using default speed and
# a slightly higher timeout (GPS modules typically update once a second).
# These are the defaults you should use for the GPS FeatherWing.
# For other boards set RX = GPS module TX, and TX = GPS module RX pins.
#uart = busio.UART(board.TX, board.RX, baudrate=9600, timeout=10)

# for a computer, use the pyserial library for uart access

# If using I2C, we'll create an I2C interface to talk to using default pins
# i2c = board.I2C()  # uses board.SCL and board.SDA
# i2c = board.STEMMA_I2C()  # For using the built-in STEMMA QT connector on a microcontroller

# Create a GPS module instance.  # Use UART/pyserial
# gps = adafruit_gps.GPS_GtopI2C(i2c, debug=False)  # Use I2C interface

# Initialize the GPS module by changing what data it sends and at what rate.
# These are NMEA extensions for PMTK_314_SET_NMEA_OUTPUT and
# PMTK_220_SET_NMEA_UPDATERATE but you can send anything from here to adjust
# the GPS module behavior:
#   https://cdn-shop.adafruit.com/datasheets/PMTK_A11.pdf

# Turn on just minimum info (RMC only, location):
# gps.send_command(b'PMTK314,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0')
# Turn off everything:
# gps.send_command(b'PMTK314,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0')
# Turn on everything (not all of it is parsed!)
# gps.send_command(b'PMTK314,1,1,1,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0')

# Or decrease to once every two seconds by doubling the millisecond value.
# Be sure to also increase your UART timeout above!
# gps.send_command(b'PMTK220,2000')
# You can also speed up the rate, but don't go too fast or else you can lose
# data during parsing.  This would be twice a second (2hz, 500ms delay):
# gps.send_command(b'PMTK220,500')


class ReadGPS(Node):
    def __init__(self):
        self.robot_id = os.getenv("ROBOT_ID")
        super().__init__(f'{self.robot_id}_gps1')
        
        self.uart = serial.Serial("/dev/ttyUSB0", baudrate=9600, timeout=10)
        self.gps_serial = adafruit_gps.GPS(self.uart, debug=False)
        # Turn on the basic GGA and RMC info (what you typically want)
        self.gps_serial.send_command(b"PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0")
        # Set update rate to once a second (1hz) which is what you typically want.
        self.gps_serial.send_command(b"PMTK220,100")

        # Exponentially moving average
        self.moving_avg_r = 0.2
        self.lat_avg = 0.0
        self.lon_avg = 0.0
        self.nodata = True

        self.gps_publisher = self.create_publisher(NavSatFix, f'/{self.robot_id}/gps1', 1)
        timer_period = 0.1 # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        msg = NavSatFix()
        self.gps_serial.update()

        lat = self.gps_serial.latitude
        lon = self.gps_serial.longitude
        status = self.gps_serial.fix_quality
        sattelites = self.gps_serial.satellites
        print(lat, lon, status, sattelites)
        
        if lat is not None and lon is not None:
            if self.nodata:
                self.lat_avg = lat
                self.lon_avg = lon
                self.nodata = False
            else:
                # Moving average
                self.lat_avg = (1 - self.moving_avg_r) * self.lat_avg + self.moving_avg_r * lat
                self.lon_avg = (1 - self.moving_avg_r) * self.lon_avg + self.moving_avg_r * lon
            msg.latitude = self.lat_avg
            msg.longitude = self.lon_avg
            msg.altitude = self.gps_serial.altitude_m
            
            msg.status.status = self.gps_serial.fix_quality
            msg.status.service = self.gps_serial.satellites
            if msg.status.status is None:
                msg.status.status = 0
            if msg.status.service is None:
                msg.status.service = 0
            msg.header.frame_id = 'primary gps'
            print(datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S"), msg.latitude, msg.longitude, msg.altitude)
        else:
            msg.status.status = 0

        try:
            self.gps_publisher.publish(msg)
        except:
            pass

def main(args=None):
    rclpy.init(args=args)
    get_gps_data = ReadGPS()
    rclpy.spin(get_gps_data)
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    get_gps_data.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
