import rclpy
from rclpy.node import Node
from pioneer_interfaces.srv import RefGPS

# ref_gps_jacobs_farm = [-121.839521, 37.260894]
# ref_gps_garage = [-121.94158, 37.35232]

class GPSReferenceServer(Node):
    def __init__(self):
        super().__init__('gps_reference_serever')
        self.ref_lon = -121.839521
        self.ref_lat = 37.260894
        self.srv = self.create_service(RefGPS, 'reference_gps', self.server_callback)
        self.get_logger().info('GPS Reference Point Server is ready.')

    def server_callback(self, request, response):
        robot_id = request.robot_id
        response.gps.longitude = self.ref_lon
        response.gps.latitude = self.ref_lat
        self.get_logger().info(f'Reference GPS sent to {robot_id}')
        
        return response

def main():
    rclpy.init()
    node = GPSReferenceServer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
