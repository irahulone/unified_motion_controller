import rclpy
from rclpy.node import Node
from std_msgs.msg import Int16
from std_srvs.srv import SetBool
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool
from std_msgs.msg import String

from .my_ros_module import PubSubManager


class GetMoveCmds(Node):

    def __init__(self, n_rover=3):
        super().__init__('rover_state_controler')

        self.N_ROVER = n_rover
        self.mode_list = ["NEU_M", "JOY_M", "NAV_M"]
        self.mode_dict = {"NEU_M": 0, "JOY_M": 1, "NAV_M": 2}
        self.rover_modeC = self.mode_list[0]

        self.joy_cmd = [0.0, 0.0]

        self.pubsub = PubSubManager(self)
        self.pubsub.create_subscription(
            Twist,
            '/joy/cmd_vel',
            self.joy_cmd_callback,
            5)
        
        self.pubsub.create_subscription(
            String,
            '/modeC',
            self.mode_callback,
            5)

        self.nav = []
        self.u = []
        for i in range(self.N_ROVER):
            self.nav.append([0.0, 0.0])
            self.u.append([0.0, 0.0])
            self.pubsub.create_subscription(
                Twist,
                f'/p{i+1}/nav_cmd_vel',
                lambda msg: self.nav_cmd_callback(msg, i),
                5)
            self.pubsub.create_publisher(
                Twist, 
                f'/p{i+1}/cmd_vel', 
                5)

        timer_period_core = 0.05  # seconds
        self.timer = self.create_timer(timer_period_core, self.core_cmd_vel_callback)


        # self.pub_rover_en = self.create_publisher(Bool, 'r4/enable', 1)
        # timer_period = 0.2  # seconds
        # self.timer = self.create_timer(timer_period, self.rover_en_callback)
        # self.i = 0
        
    def joy_cmd_callback(self, msg):
        self.joy_cmd = [msg.linear.x, msg.angular.z]
    
    def nav_cmd_callback(self, msg, i):
        self.nav[i] = [msg.linear.x, msg.angular.z]
    
    def mode_callback(self, msg):
        self.self.rover_modeC = msg.data
    
    def core_cmd_vel_callback(self):
        msg = Twist()

        if (self.rover_modeC == "JOY_M"):
            for i in range(self.N_ROVER):
                self.u[i] = self.joy_cmd
        elif (self.rover_modeC == "NAV_M"):
            for i in range(self.N_ROVER):
                self.u[i] = self.nav[i]
        elif (self.rover_modeC == "NEU_M"):
            for i in range(self.N_ROVER):
                self.u[i] = [0.0, 0.0]
        
        for i in range(self.N_ROVER):
            msg.linear.x = self.u[i][0]
            msg.angular.z = self.u[i][1]
            self.pubsub.publish(f'/p{i+1}/cmd_vel', msg)


def main(args=None):
    rclpy.init(args=args)
    sub_move_cmds = GetMoveCmds()
    rclpy.spin(sub_move_cmds)
    sub_move_cmds.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()