import rclpy

from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool, Int16
from std_msgs.msg import String
from std_msgs.msg import Float32MultiArray

from .my_ros_module import JoyBase


class JoyCmd(JoyBase):
    def __init__(self):
        super().__init__('joy_cmd')

        self.declare_parameters(
            namespace='',
            parameters=[
                ('lx', "LY"),
                ('az', "RX"),
                ('en', "LB"),
                ('rover_sel', "Y"),
                ('mode_sel', "X"),
                ('angle_sel', "A"),
                ('hardware_sim_sel', "B"),
                ('broadcast', "RB"),
                ('revolution', "cross_lr"),
                ('prismatic', "cross_ud"),
                ('n_rover', 6),
            ]
        )

        self.lx_axisN = self.axis_dict.get(self.get_parameter('lx').value)
        self.az_axisN = self.axis_dict.get(self.get_parameter('az').value)
        self.en_buttonN = self.button_dict.get(self.get_parameter('en').value)
        self.rover_sel_button = self.button_dict.get(self.get_parameter('rover_sel').value)
        self.mode_sel_button = self.button_dict.get(self.get_parameter('mode_sel').value)
        self.angle_sel_button = self.button_dict.get(self.get_parameter('angle_sel').value)
        self.hw_sel_button = self.button_dict.get(self.get_parameter('hardware_sim_sel').value)
        self.broadcast_button = self.button_dict.get(self.get_parameter('broadcast').value)
        self.N_ROVER = self.get_parameter('n_rover').value

        self.mode_list = ["NEU_M", "JOY_M", "NAV_M"]
        self.mode_dict = {"NEU_M": 0, "JOY_M": 1, "NAV_M": 2}
        self.rover_modeC = self.mode_list[0]

        self.select = 1

        self.selected_angle = 0

        self.hw_sel = True

        self.pubsub.create_publisher(Twist, '/joy/cmd_vel', 5)
        self.pubsub.create_publisher(Bool, '/joy/enable', 5)
        self.pubsub.create_publisher(Bool, '/joy/broadcast', 1)
        self.pubsub.create_publisher(Bool, '/joy/hardware', 1)
        
        self.pubsub.create_publisher(Int16, '/select_rover', 1)
        self.pubsub.create_publisher(String, '/modeC', 1)

        
        self.pubsub.create_publisher(Int16, '/joy/angle_sel', 5)
        
        self.pubsub.create_publisher(Float32MultiArray, '/joy/cross', 5)
        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.get_logger().info(
            f"JoyCmd Node Initialized\n \
            lx: {self.get_parameter('lx').value},\n \
            az: {self.get_parameter('az').value},\n \
            en: {self.get_parameter('en').value},\n \
            rover_sel: {self.get_parameter('rover_sel').value},\n \
            mode_sel: {self.get_parameter('mode_sel').value},\n \
            angle_sel: {self.get_parameter('angle_sel').value},\n \
            hardware_sel: {self.get_parameter('hardware_sim_sel').value},\n \
            broadcast: {self.get_parameter('broadcast').value},\n \
            revolution: {self.get_parameter('revolution').value},\n \
            prismatic: {self.get_parameter('prismatic').value}\n"
        )

    
    def joy_callback(self, msg):
        _toggle = self.joy_toggle(msg)
        gain = 0.5

        j_lx = msg.axes[self.lx_axisN]
        j_az = msg.axes[self.az_axisN]

        val = Twist()           # For selected rover
        en_state = Bool()       # For selected rover
        empty_twist = Twist()   # For other rovers
        false_state = Bool()    # For other rovers
        
        empty_twist.linear.x = 0.0
        empty_twist.angular.z = 0.0
        false_state.data = False

        # Toggle select rover
        if _toggle[self.rover_sel_button] == 1:
            self.select = self.select%self.N_ROVER + 1
            self.get_logger().info(f"Sel Button Toggled: {self.select}")

        if _toggle[self.hw_sel_button] == 1:
            self.hw_sel = not self.hw_sel
            self.get_logger().info(f"HW/Sim Button Toggled: { 'HW' if self.hw_sel else 'Sim' }")

        if _toggle[self.mode_sel_button] == 1:
            self.get_logger().info(f"Mode Button Toggled: {self.rover_modeC} to {self.mode_list[(self.mode_dict[self.rover_modeC] + 1) % 3]}")
            self.rover_modeC = self.mode_list[(self.mode_dict[self.rover_modeC] + 1) % 3]
        
        if _toggle[self.angle_sel_button] == 1:
            self.get_logger().info(f"Angle Button Toggled: {self.selected_angle} to {(self.selected_angle+90)%360}")
            self.selected_angle = (self.selected_angle+90)%360
            angle_msg = Int16()
            angle_msg.data = self.selected_angle
            self.pubsub.publish('/joy/angle_sel', angle_msg)
            
        broadcast_msg = Bool()
        if msg.buttons[self.broadcast_button] == 1:
            broadcast_msg.data = True
        else:
            broadcast_msg.data = False
        self.pubsub.publish('/joy/broadcast', broadcast_msg)

        if msg.buttons[self.en_buttonN] == 1:
            en_state.data = True
            val.linear.x = 0.5*j_lx*gain
            val.angular.z = 0.4*j_az*gain
            self.pubsub.publish('/joy/cmd_vel', val)
            self.pubsub.publish('/joy/enable', en_state)
        else:
            self.pubsub.publish('/joy/cmd_vel', empty_twist)
            self.pubsub.publish('/joy/enable', false_state)
        
        msg_formation = Float32MultiArray()
        msg_formation.data = [msg.axes[self.axis_dict[self.get_parameter('prismatic').value]], msg.axes[self.axis_dict[self.get_parameter('revolution').value]]]
        self.pubsub.publish('/joy/cross', msg_formation)

    def timer_callback(self):
        select_msg = Int16()
        select_msg.data = self.select
        self.pubsub.publish('/select_rover', select_msg)

        mode_msg = String()
        mode_msg.data = self.rover_modeC
        self.pubsub.publish('/modeC', mode_msg)   

        hardware_msg = Bool()
        hardware_msg.data = self.hw_sel
        self.pubsub.publish('/joy/hardware', hardware_msg)      


def main(args=None):
    rclpy.init(args=args)
    joy_handle = JoyCmd()
    rclpy.spin(joy_handle)
    joy_handle.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

#ros2 run joy joy_node
