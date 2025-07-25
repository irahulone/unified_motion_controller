from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from std_msgs.msg import Float32

# Custom submodules
from roboteq_core.status import Status


class Publisher(Node):
    """
    @class: Publish any stats regarding batteries
    """
    def __init__(self, connection, user):
        super().__init__('roboteq_batt_volt_pub')
        self.get_stat = Status(connection)
        self.publisher = self.create_publisher(Float32, f"{user}/battery_voltage", 10)
        battery_voltage_period = 1  # seconds
        self.timer = self.create_timer(battery_voltage_period,
                                       self.battery_voltage_callback,
                                       callback_group=MutuallyExclusiveCallbackGroup())
        self.i = 0

    def battery_voltage_callback(self):
        """
        Read the 12v battery voltage and publish
        :return:
        """
        msg = Float32()
        msg.data = self.get_stat.read_battery_12v()
        self.publisher.publish(msg)
        # self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1



