from rclpy.node import Node
from std_msgs.msg import String
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

from roboteq_core.connect import handle_shell_request


class Publisher(Node):
    """
    @class: Publish any stats regarding vehicle computer
    """
    def __init__(self, user):
        super().__init__('shell_user_ip_pub')
        self.publisher = self.create_publisher(String, f"{user}/user_ip", 10)
        period = 10  # seconds
        self.timer = self.create_timer(period,
                                       self.user_ip_callback,
                                       callback_group=MutuallyExclusiveCallbackGroup())
        self.shell_cmds = {
            "ip": 'ifconfig | grep 192 | awk \'{print $2}\'',
            "user": 'echo $USER'
        }
        self.i = 0

    def user_ip_callback(self):
        """
        Get IP of vehicle computer

        awk command splits string by whitespace and NF flag contains the number of fields
        Therefore, '{print $NF}' always gets the last field of a string of words
        :return:
        """

        msg = String()
        ip = handle_shell_request(self.shell_cmds["ip"])
        user = handle_shell_request(self.shell_cmds["user"])
        
        msg.data = user + '@' + ip

        self.publisher.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1

