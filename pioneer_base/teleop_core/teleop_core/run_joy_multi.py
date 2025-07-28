# NOTE: Not recommend for use this now.
# Please use run_joy_cmd and demux by following:
# $ ros2 launch teleop_core teleop_node_multi.launch.py 


import rclpy

from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool, Int16

from .my_ros_module import JoyBase


class JoyMulti(JoyBase):

    def __init__(self, n_rover=6):
        super().__init__('multi_teleop')

        self.lx_axisN = self.axis_dict.get("LY")
        self.az_axisN = self.axis_dict.get("RX")
        self.en_buttonN = self.button_dict.get("LB")
        self.toggle_button = self.button_dict.get("Y")

        self.N_ROVER = n_rover
        self.select = 1

        for i in range(self.N_ROVER):
            self.pubsub.create_publisher(Twist, f'/p{i+1}/cmd_vel', 5)
            self.pubsub.create_publisher(Bool, f'/p{i+1}/enable', 5)
        
        self.pubsub.create_publisher(Int16, '/select_rover', 1)
        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.timer_callback)
    
    def joy_callback(self, msg):
        _toggle = self.joy_toggle(msg)

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
        if _toggle[self.toggle_button] == 1:
            self.select = self.select%self.N_ROVER + 1

        for i in range(self.N_ROVER):
            if msg.buttons[self.en_buttonN] == 1 and self.select == i+1:
                en_state.data = True
                val.linear.x = 0.7*j_lx
                val.angular.z = 0.5*j_az
                self.pubsub.publish(f'/p{i+1}/cmd_vel', val)
                self.pubsub.publish(f'/p{i+1}/enable', en_state)
            else:
                self.pubsub.publish(f'/p{i+1}/cmd_vel', empty_twist)
                self.pubsub.publish(f'/p{i+1}/enable', false_state)

    def timer_callback(self):
        select_msg = Int16()
        select_msg.data = self.select
        self.pubsub.publish('/select_rover', select_msg)               


def main(args=None):
    rclpy.init(args=args)
    joy_handle = JoyMulti()
    rclpy.spin(joy_handle)
    joy_handle.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

#ros2 run joy joy_node
