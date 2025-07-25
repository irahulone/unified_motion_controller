import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from geometry_msgs.msg import Quaternion
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Int16MultiArray

import time
import board
import adafruit_bno055
import yaml
import os

# If you are going to use UART uncomment these lines
# uart = board.UART()
# self.sensor = adafruit_bno055.BNO055_UART(uart)


class ReadImu(Node):
    def __init__(self):
        self.robot_id = os.getenv("ROBOT_ID")
        self.username = os.getenv("USER")
        super().__init__(f'{self.robot_id}_imu')

        imu_offset = os.getenv("IMU_OFFSET")
        if imu_offset is None:
            self.heading_offset = 0
        else:
            self.heading_offset = int(os.getenv("IMU_OFFSET"))

        self.i2c = board.I2C()  # uses board.SCL and board.SDA
        # i2c = board.STEMMA_I2C()  # For using the built-in STEMMA QT connector on a microcontroller
        self.sensor = adafruit_bno055.BNO055_I2C(self.i2c)
        self.declare_parameters(
            namespace='',
            parameters=[
                ('calibrateIMU', 0),
                ('calibOffsetsRadii', [3, 146, 3, 232, 0, 0, 255, 255, 255, 255, 239, 131, 252, 100, 254, 79, 255, 227, 0, 19, 255, 204]),
                ('calibFileLoc', f"/home/{self.username}/ros_ws/src/imu_core/config/paramsIMU.yaml")
            ]
        )
        self.publisher_quaternion = self.create_publisher(Quaternion, f'{self.robot_id}/imu/quaternion', 1)
        self.publisher_euler = self.create_publisher(Float32MultiArray, f'{self.robot_id}/imu/eulerAngle', 3)
        self.publisher_calib = self.create_publisher(Int16MultiArray, f'{self.robot_id}/imu/calibInfo', 4)

        # Create a subscription to its own topic with a callback function 
        #This is so it can listen for commands to start/set/store calibration
        self.subscription = self.create_subscription(
            Int16MultiArray,
            f'{self.robot_id}/imu/calibCom',
            self.calib_cmd_callback, 
            5)
        self.subscription  

        #Set Calibration if calibIMU is True
        if(self.get_parameter('calibrateIMU').get_parameter_value()):
            calib_data_param = self.get_parameter('calibOffsetsRadii').get_parameter_value().integer_array_value
            print(self.get_parameter('calibrateIMU').get_parameter_value())
            self.set_calibration(calib_data_param)

        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def calib_cmd_callback(self, msg:Int16MultiArray):
        run_calib_reset_device = msg.data[0]
        set_calib_param = msg.data[1]
        store_calib = msg.data[2]

        # Only do something if command
        if(run_calib_reset_device):
            # Run Calibration by resetting device
            self.sensor = adafruit_bno055.BNO055_I2C(self.i2c)
        elif(set_calib_param):
            # Set Calibration Offsets/Radii from parameters file
            calib_data = self.get_parameter('calibOffsetsRadii').get_parameter_value().integer_array_value
            self.set_calibration(calib_data)
        elif(store_calib):
            #Store current calib offsets/radii in parameters file
            #Also set parameter CalibIMU to True so offsets will be used next time
            calib_data=self.get_calibration(self.sensor)
            #Store Data
            calib_file = self.get_parameter('calibFileLoc')
            calib_file = calib_file.get_parameter_value().string_value
            data = {f'{self.robot_id}_imu':{'ros__parameters':{'calibrateIMU': 1, 'calibOffsetsRadii':calib_data,'calibFileLoc':calib_file}}}

            with open(calib_file,'w',) as yaml_file:
                yaml.dump(data,yaml_file,default_flow_style=None)
                yaml_file.close()
             
    def set_calibration(self, calib_data:list):
        """
        Set the sensor's calibration data using a list of 22 bytes that
        represent the sensor offsets and calibration data.  This data should be
        a value that was previously retrieved with get_calibration (and then
        perhaps persisted to disk or other location until needed again).
        """
        # Check that 22 bytes were passed in with calibration data.
        if calib_data is None or len(calib_data) != 22:
            print(calib_data)
            raise ValueError('Expected a list of 22 bytes for calibration data.')
        
        # Switch to configuration mode, as mentioned in section 3.10.4 of datasheet.
        self.sensor._write_register(adafruit_bno055._MODE_REGISTER,adafruit_bno055.CONFIG_MODE)  # Empirically necessary
        time.sleep(0.02)  # Datasheet table 3.6

        # Write 22 bytes of calibration data.
        register_addr = 0x6A #Start with Magnometer radius MSB register

        for i in range(22):
            self.sensor._write_register(register_addr,calib_data[i])
            #Update register Address:
            register_addr-=0x1

        # Go back to normal operation mode.
        time.sleep(0.01)  # Table 3.6
        self.sensor._write_register(adafruit_bno055._MODE_REGISTER,adafruit_bno055.NDOF_MODE)

    def get_calibration(self):
            """
            Return the sensor's calibration data and return it as an array of
            22 bytes. Can be saved and then reloaded with the set_calibration function
            to quickly calibrate from a previously calculated set of calibration data.
            """
            # Switch to configuration mode, as mentioned in section 3.10.4 of datasheet.
            self.sensor._write_register(adafruit_bno055._MODE_REGISTER,adafruit_bno055.CONFIG_MODE)  # Empirically necessary
            time.sleep(0.02)  # Datasheet table 3.6

            # Read the 22 bytes of calibration data and put it in a list
            calib_data = []
            register_addr = 0x6A #Start with Magnometer radius MSB register

            for _ in range(22):
                calib_data.append(self.sensor._read_register(register_addr))
                #Update register Address:
                register_addr-=0x1

            # Go back to normal operation mode.
            time.sleep(0.01)  # Table 3.6
            self.sensor._write_register(adafruit_bno055._MODE_REGISTER,adafruit_bno055.NDOF_MODE)
            return calib_data

    def timer_callback(self):
        #Publish Quaternion Data
        msg_quat = Quaternion()
        msg_quat.x, msg_quat.y, msg_quat.z, msg_quat.w = self.sensor.quaternion[0:4]
        self.publisher_quaternion.publish(msg_quat)

        #Publish Euler Angle Data
        msg_euler = Float32MultiArray()
        msg_euler.data = self.sensor.euler
        msg_euler.data[0] = (msg_euler.data[0] + self.heading_offset) % 360
        self.publisher_euler.publish(msg_euler)

        #Publish Calibration Status
        #One Array: [run_calib_reset_device setCalibrationParam store_calib sysCalib gyroCalib accelCalib magCalib]
        #Array Data Range: [0/1 0/1 0/1 0-3 0-3 0-3 0-3]

        msg_calib = Int16MultiArray()
        # msg_calib.data = [0,0,0]
        # msg_calib.data.extend(self.sensor.calibration_status)
        msg_calib.data = self.sensor.calibration_status
        self.publisher_calib.publish(msg_calib)


def main(args=None):
    rclpy.init(args=args)
    get_imu_data = ReadImu()
    rclpy.spin(get_imu_data)
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    get_imu_data.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
