from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

import logging
logging.basicConfig(level=logging.WARNING)
logging.getLogger(__name__)


# Custom submodules
from roboteq_core.status import Status

QUEUESIZE = 10


class Move(Node):
    """
    @class: Any movement we would like to send to the controller or keep track of
    """
    def __init__(self, connection, topic, msg_type):
        super().__init__("roboteq_move_subscriber")
        # ROBOTEQ commands SDC2160 controller
        self.move_cmds = {
            "move_wheels": "!M {} {}_"
        }
        self.connect = connection
        self.get_stat = Status(self.connect)

        self.create_subscription(
            msg_type,
            topic,
            self.move_motors_callback,
            QUEUESIZE,
            callback_group=MutuallyExclusiveCallbackGroup()
        )

    def move_motors_callback(self, msg):
        if type(msg.data) is float:
            cmd = self.move_cmds["move_wheels"].format(msg.data, msg.data)
        else:
            cmd = self.move_cmds["move_wheels"].format(msg.data[0], msg.data[1])
        self.connect.send_write_command(cmd)

