import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import String
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import Twist
import random
import math
from teleop_core.my_ros_module import PubSubManager
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import threading

# Global variable to store the node instance
node_instance = None

class ClusterFeedbackNode(Node):
    def __init__(self):
        super().__init__('cluster_feedback')
        self.declare_parameters(
            namespace='',
            parameters=[
                ('robot_id_list', ["p0"]),
                ('reference_lat', 37.774932),  # Example reference latitude
                ('reference_lon', -122.419467)  # Example reference longitude
            ]
        )
        self.robot_id_list = self.get_parameter('robot_id_list').value
        self.reference_lat = self.get_parameter('reference_lat').value
        self.reference_lon = self.get_parameter('reference_lon').value
        self.plotSize = 10.0
        self.pubsub = PubSubManager(self)
        self.gps_positions = [[self.reference_lat, self.reference_lon],[self.reference_lat-math.pow(10, -4), self.reference_lon-math.pow(10, -4)],[self.reference_lat-math.pow(10, -4), self.reference_lon+math.pow(10, -4)]]  # Initialize GPS positions
        self.xy_positions = self.calculate_xy_positions()
        self.velocities = {robot_id: (0.0, 0.0) for robot_id in self.robot_id_list}  # Initialize velocities
        
        for robot_id in self.robot_id_list:
            self.pubsub.create_publisher(Float32MultiArray, f'/{robot_id}/imu/eulerAngle', 10)
            self.pubsub.create_publisher(NavSatFix, f'/{robot_id}/gps1', 10)
            self.pubsub.create_subscription(Twist, f'/{robot_id}/cmd_vel', lambda msg, robot_id=robot_id: self.cmd_vel_callback(msg, robot_id), 10)
        
        self.timer = self.create_timer(0.1, self.publish_feedback)  # 10 Hz
        self.start = self.create_timer(2, self.startCluster)

    def startCluster(self):
        self.pubsub.create_publisher(String, '/joy/mode', 5)
        start = String()
        start.data = "cluster"
        self.pubsub.publish('/joy/mode', start)
        self.start.cancel()
    def calculate_gps_positions(self):
        positions = []
        distance = 20  # Distance in meters
        R = 6371000  # Earth radius in meters

        for i in range(len(self.robot_id_list)):
            # Calculate the offset in radians
            dlat = distance * i / R
            dlon = distance * i / (R * math.cos(math.radians(self.reference_lat)))

            # Calculate the new latitude and longitude
            new_lat = self.reference_lat + math.degrees(dlat)
            new_lon = self.reference_lon + math.degrees(dlon)
            positions.append([new_lat, new_lon])

        return positions

    def calculate_xy_positions(self):
        xy_positions = []
        for lat, lon in self.gps_positions:
            x, y = self.gps_to_xy(self.reference_lat, self.reference_lon, lat, lon)
            xy_positions.append([x, y])
            self.get_logger().info(f"Calculated XY position: {x}, {y}")
        return xy_positions

    def gps_to_xy(self, lat1, lon1, lat2, lon2):
        R = 6371000   # Earth radius in meters
        # Convert degrees to radians
        lat1_rad = math.radians(lat1)
        lon1_rad = math.radians(lon1)
        lat2_rad = math.radians(lat2)
        lon2_rad = math.radians(lon2)

        dlat = lat2_rad - lat1_rad
        dlon = lon2_rad - lon1_rad
        a = math.sin(dlat / 2)**2 + math.cos(lat1_rad) * math.cos(lat2_rad) * math.sin(dlon / 2)**2
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
        distance = R * c

        # Calculate bearing
        y = math.sin(dlon) * math.cos(lat2_rad)
        x = math.cos(lat1_rad) * math.sin(lat2_rad) - math.sin(lat1_rad) * math.cos(lat2_rad) * math.cos(dlon)
        bearing = math.atan2(y, x)
        x = distance * math.sin(bearing)
        y = distance * math.cos(bearing)

        return x, y

    def cmd_vel_callback(self, msg, robot_id):
        # Update the velocity for the given robot
        self.velocities[robot_id] = (msg.linear.x, msg.linear.y)
        #self.get_logger().info(f"Updated velocities for {robot_id}: Linear X: {msg.linear.x}, Linear Y: {msg.linear.y}")

    def publish_feedback(self):
        #self.get_logger().info(f"Updated XY position for {self.robot_id_list[0]}: {self.xy_positions[0]}")
        for i, robot_id in enumerate(self.robot_id_list):
            #self.xy_positions[i] = self.gps_to_xy(self.reference_lat, self.reference_lon, self.gps_positions[i][0], self.gps_positions[i][1])
            temp = [self.xy_positions[i][0], self.xy_positions[i][1]]
            
            # Simulate movement based on velocity
            linear_vel_x, linear_vel_y = self.velocities[robot_id]
            self.gps_positions[i][0] += linear_vel_y * 0.1 / 111320  # Approximate conversion from meters to degrees latitude
            self.gps_positions[i][1] += linear_vel_x * 0.1 / (111320 * math.cos(math.radians(self.gps_positions[i][0])))  # Approximate conversion from meters to degrees longitude
            
            # Update xy positions
            self.xy_positions[i] = self.gps_to_xy(self.reference_lat, self.reference_lon, self.gps_positions[i][0], self.gps_positions[i][1])
            #self.get_logger().info(f"Actual position change for {robot_id}: {self.xy_positions[i][0]- temp[0]}, {self.xy_positions[i][1] - temp[1]} expected: {linear_vel_x * 0.1}, {linear_vel_y * 0.1 }")

            # Create and publish IMU data
            imu_msg = Float32MultiArray()
            imu_msg.data = [0.0, 0.0, 0.0]
            self.pubsub.publish(f'/{robot_id}/imu/eulerAngle', imu_msg)

            # Create and publish GPS data
            gps_msg = NavSatFix()
            gps_msg.latitude = float(self.gps_positions[i][0])
            gps_msg.longitude = float(self.gps_positions[i][1])
            gps_msg.altitude = float(0)  # Fixed altitude
            self.pubsub.publish(f'/{robot_id}/gps1', gps_msg)

    def update_plot(self, scatter, ax):
        xs = [pos[0] for pos in self.xy_positions]
        ys = [pos[1] for pos in self.xy_positions]
        #self.get_logger().info(f"Publishing XY positions: {xs}, {ys}")
        scatter.set_offsets(list(zip(xs, ys)))
        
        # Calculate the center and size of the plot
        x_center = (max(xs) + min(xs)) / 2
        y_center = (max(ys) + min(ys)) / 2
        plot_size = max(max(xs) - min(xs), max(ys) - min(ys)) * 1.1  # Add some padding

        # Set the limits to ensure the plot is square
        ax.set_xlim(x_center - plot_size / 2, x_center + plot_size / 2)
        ax.set_ylim(y_center - plot_size / 2, y_center + plot_size / 2)



def run_ros_node():
    global node_instance
    rclpy.init()
    node_instance = ClusterFeedbackNode()
    try:
        rclpy.spin(node_instance)
    except KeyboardInterrupt:
        pass
    node_instance.destroy_node()
    rclpy.shutdown()

def main():
    global node_instance
    # Start the ROS node in a separate thread
    ros_thread = threading.Thread(target=run_ros_node)
    ros_thread.start()

    # Set up the plot in the main thread
    fig, ax = plt.subplots()
    scatter = ax.scatter([], [])
    ani = FuncAnimation(fig, lambda frame: update_plot(frame, scatter, ax), interval=100)

    # Start the Matplotlib GUI in the main thread
    plt.show()

def update_plot(frame, scatter, ax):
    global node_instance
    # Update the plot with the current positions
    if node_instance:
        node_instance.update_plot(scatter, ax)

if __name__ == '__main__':
    main()