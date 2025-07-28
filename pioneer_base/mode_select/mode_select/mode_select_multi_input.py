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
        self.declare_parameters(
            namespace='',
            parameters=[
                ('robot_id_list', ["p0"])
            ]
        )
        
        robot_id_list = self.get_parameter('robot_id_list').value
        self.get_logger().info(f"robot_id_list: {robot_id_list}")
        if len(robot_id_list) == 1 and robot_id_list[0] == "p0":
            robot_id_list = None
        self.robot_id_list = []
        if robot_id_list is None:
            self.n_rover = n_rover
            for i in range(self.n_rover):
                self.robot_id_list.append(f"{self.robot_id_list[i]}")
        elif isinstance(robot_id_list, int):
            self.n_rover = n_rover
            for i in range(self.n_rover):
                self.robot_id_list.append(f"p{i+1+robot_id_list}")
        elif isinstance(robot_id_list, list):
            self.n_rover = len(robot_id_list)
            if isinstance(robot_id_list[0], int):
                for i in range(self.n_rover):
                    self.robot_id_list.append(f"p{robot_id_list[i]}")
            else:
                self.robot_id_list = robot_id_list
        else:
            print("ROBOT_ID_LIST ERROR")
            return
        self.get_logger().info(f"ROVER: {self.robot_id_list} N: {self.n_rover}")

        self.mode_list = ["NEU_M", "JOY_M", "NAV_M"]
        self.mode_dict = {"NEU_M": 0, "JOY_M": 1, "NAV_M": 2}
        self.rover_modeC = self.mode_list[0]

        self.joy_cmd = [0.0, 0.0]
        self.joy_cmd_demux = []
        for _ in range(self.n_rover):
            self.joy_cmd_demux.append([0.0, 0.0])

        self.pubsub = PubSubManager(self)
        '''
        self.pubsub.create_subscription(
            Twist,
            '/joy/cmd_vel',
            self.joy_cmd_callback,
            5)
        '''
        
        self.pubsub.create_subscription(
            String,
            '/modeC',
            self.mode_callback,
            5)

        self.nav = []
        self.u = []
        for i in range(self.n_rover):
            self.nav.append([0.0, 0.0])
            self.u.append([0.0, 0.0])
            self.pubsub.create_subscription(
                Twist,
                f'/joy/{self.robot_id_list[i]}/cmd_vel',
                lambda msg: self.joy_cmd_callback(msg, i),
                5)
            self.pubsub.create_subscription(
                Twist,
                f'/nav/{self.robot_id_list[i]}/cmd_vel',
                lambda msg: self.nav_cmd_callback(msg, i),
                5)
            self.pubsub.create_publisher(
                Twist, 
                f'/{self.robot_id_list[i]}/cmd_vel', 
                5)

        timer_period_core = 0.05  # seconds
        self.timer = self.create_timer(timer_period_core, self.core_cmd_vel_callback)


        # self.pub_rover_en = self.create_publisher(Bool, 'r4/enable', 1)
        # timer_period = 0.2  # seconds
        # self.timer = self.create_timer(timer_period, self.rover_en_callback)
        # self.i = 0
        
    def joy_cmd_callback(self, msg, i):
        self.joy_cmd_demux[i] = [msg.linear.x, msg.angular.z]
    
    def nav_cmd_callback(self, msg, i):
        self.nav[i] = [msg.linear.x, msg.angular.z]
    
    def mode_callback(self, msg):
        self.rover_modeC = msg.data
    
    def core_cmd_vel_callback(self):
        msg = Twist()

        if (self.rover_modeC == "JOY_M"):
            for i in range(self.n_rover):
                self.u[i] = self.joy_cmd_demux[i]
        elif (self.rover_modeC == "NAV_M"):
            for i in range(self.n_rover):
                self.u[i] = self.nav[i]
        elif (self.rover_modeC == "NEU_M"):
            for i in range(self.n_rover):
                self.u[i] = [0.0, 0.0]
        
        for i in range(self.n_rover):
            msg.linear.x = self.u[i][0]
            msg.angular.z = self.u[i][1]
            self.pubsub.publish(f'/{self.robot_id_list[i]}/cmd_vel', msg)


def main(args=None):
    rclpy.init(args=args)
    sub_move_cmds = GetMoveCmds()
    rclpy.spin(sub_move_cmds)
    sub_move_cmds.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()