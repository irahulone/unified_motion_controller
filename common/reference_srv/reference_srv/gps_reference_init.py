import rclpy
from rclpy.node import Node
from pioneer_interfaces.srv import RefGPS
import sys

REFERENCE_FILE_PATH = './common/reference_srv/ref/reference.txt'

class GPSReferenceRequest(Node):
    def __init__(self):
        super().__init__('reference_gps_request')
        self.cli = self.create_client(RefGPS, 'reference_gps')

        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting...')
        
        self.req = RefGPS.Request()
        self.future = self.cli.call_async(self.req)
        self.future.add_done_callback(self.callback)
    
    def callback(self, future):
        try:
            lat = future.result().gps.latitude
            lon = future.result().gps.longitude
            self.get_logger().info('Reference GPS received:')
            with open(REFERENCE_FILE_PATH, 'w') as f:
                f.write(f'{lat},{lon}')
            self.get_logger().info('Reference GPS saved to file.')
            sys.exit(0)  # 正常終了
        except Exception as e:
            self.get_logger().error(f'Fail to connect to the server: {e}')
            sys.exit(1)


def main():
    rclpy.init()
    node = GPSReferenceRequest()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
