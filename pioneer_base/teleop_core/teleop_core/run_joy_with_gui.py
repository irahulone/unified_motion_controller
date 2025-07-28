import sys
import rclpy
from rclpy.node import Node
import math
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool, Int16, String, Float32MultiArray

from .my_ros_module import JoyBase

from PyQt6.QtWidgets import (
    QApplication, QMainWindow, QLabel, QVBoxLayout, QHBoxLayout, QWidget,
    QSpinBox, QComboBox, QCheckBox, QDoubleSpinBox
)
from PyQt6.QtCore import QTimer


class JoyCmd(JoyBase):
    def __init__(self):
        super().__init__('joy_cmd')

        self.declare_parameters(
            namespace='',
            parameters=[
                ('lx', "LX"),
                ('ly', "LY"),
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
        self.ly_axisN = self.axis_dict.get(self.get_parameter('ly').value)
        self.az_axisN = self.axis_dict.get(self.get_parameter('az').value)
        self.en_buttonN = self.button_dict.get(self.get_parameter('en').value)
        self.rover_sel_button = self.button_dict.get(self.get_parameter('rover_sel').value)
        self.mode_sel_button = self.button_dict.get(self.get_parameter('mode_sel').value)
        self.angle_sel_button = self.button_dict.get(self.get_parameter('angle_sel').value)
        self.hw_sel_button = self.button_dict.get(self.get_parameter('hardware_sim_sel').value)
        self.broadcast_button = self.button_dict.get(self.get_parameter('broadcast').value)
        self.N_ROVER = self.get_parameter('n_rover').value
        self.cluster_x = 0.0
        self.cluster_y = 0.0
        self.cluster_t = 0.0
        self.cluster_b = math.pi / 3
        self.cluster_p = 10.0
        self.cluster_q = 10.0
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
        self.pubsub.create_publisher(Float32MultiArray, '/cluster_params', 5)
        self.pubsub.create_publisher(Float32MultiArray, '/cluster_desired', 5)

        self.pubsub.create_subscription(Float32MultiArray, '/cluster_desired', self.cluster_desired_callback, 5)

        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.get_logger().info(
            f"JoyCmd Node Initialized\n"
            f"lx: {self.get_parameter('lx').value},\n"
            f"az: {self.get_parameter('az').value},\n"
            f"en: {self.get_parameter('en').value},\n"
            f"rover_sel: {self.get_parameter('rover_sel').value},\n"
            f"mode_sel: {self.get_parameter('mode_sel').value},\n"
            f"angle_sel: {self.get_parameter('angle_sel').value},\n"
            f"hardware_sel: {self.get_parameter('hardware_sim_sel').value},\n"
            f"broadcast: {self.get_parameter('broadcast').value},\n"
            f"revolution: {self.get_parameter('revolution').value},\n"
            f"prismatic: {self.get_parameter('prismatic').value}\n"
        )

    def joy_callback(self, msg):
        _toggle = self.joy_toggle(msg)
        j_lx = msg.axes[self.lx_axisN]
        j_ly = msg.axes[self.ly_axisN]
        j_az = msg.axes[self.az_axisN]
        #self.get_logger().info(f"lx: {j_lx}, az: {j_az}, stick: {msg.axes[0]}, {msg.axes[1]}, {msg.axes[2]}, {msg.axes[3]}")

        val = Twist()
        en_state = Bool()
        empty_twist = Twist()
        false_state = Bool() 
        
        empty_twist.linear.x = 0.0
        empty_twist.angular.z = 0.0
        false_state.data = False

        if _toggle[self.rover_sel_button] == 1:
            self.select = self.select % self.N_ROVER + 1
            self.get_logger().info(f"Sel Button Toggled: {self.select}")

        if _toggle[self.hw_sel_button] == 1:
            self.hw_sel = not self.hw_sel
            self.get_logger().info(f"HW/Sim Button Toggled: {'HW' if self.hw_sel else 'Sim'}")

        if _toggle[self.mode_sel_button] == 1:
            new_mode = self.mode_list[(self.mode_dict[self.rover_modeC] + 1) % 3]
            self.get_logger().info(f"Mode Button Toggled: {self.rover_modeC} to {new_mode}")
            self.rover_modeC = new_mode
        
        if _toggle[self.angle_sel_button] == 1:
            new_angle = (self.selected_angle + 90) % 360
            self.get_logger().info(f"Angle Button Toggled: {self.selected_angle} to {new_angle}")
            self.selected_angle = new_angle
            angle_msg = Int16()
            angle_msg.data = self.selected_angle
            self.pubsub.publish('/joy/angle_sel', angle_msg)
            
        broadcast_msg = Bool()
        broadcast_msg.data = True if msg.buttons[self.broadcast_button] == 1 else False
        self.pubsub.publish('/joy/broadcast', broadcast_msg)

        if msg.buttons[self.en_buttonN] == 1:
            en_state.data = True
            val.linear.x = j_lx
            val.linear.y = j_ly
            val.angular.z = j_az
            self.pubsub.publish('/joy/cmd_vel', val)
            self.pubsub.publish('/joy/enable', en_state)
        else:
            self.pubsub.publish('/joy/cmd_vel', empty_twist)
            self.pubsub.publish('/joy/enable', false_state)
        
        msg_formation = Float32MultiArray()
        prismatic_val = msg.axes[self.axis_dict[self.get_parameter('prismatic').value]]
        revolution_val = msg.axes[self.axis_dict[self.get_parameter('revolution').value]]
        msg_formation.data = [prismatic_val, revolution_val]
        self.pubsub.publish('/joy/cross', msg_formation)
    
    def cluster_desired_callback(self, msg):
        self.cluster_x = msg.data[0]
        self.cluster_y = msg.data[1]
        self.cluster_t = msg.data[2]

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


class StatusWindow(QMainWindow):
    def __init__(self, node):
        super().__init__()
        self.node = node
        self.setWindowTitle("JoyCmd Status")

        central_widget = QWidget()
        main_layout = QHBoxLayout()

        left_layout = QVBoxLayout()
        status_header = QLabel("Status")
        left_layout.addWidget(status_header)
        self.status_labels = {
            "selected_rover": QLabel("Selected Rover: N/A"),
            "mode": QLabel("Mode: N/A"),
            "selected_angle": QLabel("Selected Angle: N/A"),
            "hardware": QLabel("Hardware: N/A"),
            "cluster_x": QLabel(f"Cluster Xc: {self.node.cluster_x}"),
            "cluster_y": QLabel(f"Cluster Yc: {self.node.cluster_y}"),
            "cluster_t": QLabel(f"Cluster Tc: {self.node.cluster_t}"),
            "cluster_p": QLabel(f"Cluster P: {self.node.cluster_p}"),
            "cluster_q": QLabel(f"Cluster Q: {self.node.cluster_q}"),
            "cluster_b": QLabel(f"Cluster B: {self.node.cluster_b}"),
        }
        for label in self.status_labels.values():
            left_layout.addWidget(label)

        modify_header = QLabel("Modify Status")
        left_layout.addWidget(modify_header)

        self.rover_spin = QSpinBox()
        self.rover_spin.setMinimum(1)
        self.rover_spin.setMaximum(self.node.N_ROVER)
        self.rover_spin.setValue(self.node.select)
        self.rover_spin.valueChanged.connect(self.update_rover)
        left_layout.addWidget(QLabel("Select Rover:"))
        left_layout.addWidget(self.rover_spin)

        self.mode_combo = QComboBox()
        self.mode_combo.addItems(self.node.mode_list)
        current_mode_index = self.node.mode_dict[self.node.rover_modeC]
        self.mode_combo.setCurrentIndex(current_mode_index)
        self.mode_combo.currentTextChanged.connect(self.update_mode)
        left_layout.addWidget(QLabel("Mode:"))
        left_layout.addWidget(self.mode_combo)

        self.angle_combo = QComboBox()
        angles = ["0", "90", "180", "270"]
        self.angle_combo.addItems(angles)
        self.angle_combo.setCurrentIndex(self.node.selected_angle // 90)
        self.angle_combo.currentTextChanged.connect(self.update_angle)
        left_layout.addWidget(QLabel("Selected Angle:"))
        left_layout.addWidget(self.angle_combo)

        self.hw_checkbox = QCheckBox("HW Mode")
        self.hw_checkbox.setChecked(self.node.hw_sel)
        self.hw_checkbox.toggled.connect(self.update_hw)
        left_layout.addWidget(self.hw_checkbox)

        # Add fields for cluster parameters P, Q, and B
        self.x_spin = QDoubleSpinBox()
        self.x_spin.setMinimum(-100.0)
        self.x_spin.setMaximum(100.0)
        self.x_spin.setValue(self.node.cluster_x)
        self.x_spin.valueChanged.connect(self.update_cluster_x)
        left_layout.addWidget(QLabel("Cluster Xc(m):"))
        left_layout.addWidget(self.x_spin)

        self.y_spin = QDoubleSpinBox()
        self.y_spin.setMinimum(-100.0)
        self.y_spin.setMaximum(100.0)
        self.y_spin.setValue(self.node.cluster_y)
        self.y_spin.valueChanged.connect(self.update_cluster_y)
        left_layout.addWidget(QLabel("Cluster Yc(m):"))
        left_layout.addWidget(self.y_spin)

        self.t_spin = QDoubleSpinBox()
        self.t_spin.setMinimum(-10 * math.pi)
        self.t_spin.setMaximum(10 * math.pi)
        self.t_spin.setValue(self.node.cluster_t)
        self.t_spin.valueChanged.connect(self.update_cluster_t)
        left_layout.addWidget(QLabel("Cluster Tc(rad):"))
        left_layout.addWidget(self.t_spin)

        self.p_spin = QDoubleSpinBox()
        self.p_spin.setMinimum(0.0)
        self.p_spin.setMaximum(100.0)
        self.p_spin.setValue(self.node.cluster_p)
        self.p_spin.valueChanged.connect(self.update_cluster_p)
        left_layout.addWidget(QLabel("Cluster P(m):"))
        left_layout.addWidget(self.p_spin)

        self.q_spin = QDoubleSpinBox()
        self.q_spin.setMinimum(0.0)
        self.q_spin.setMaximum(100.0)
        self.q_spin.setValue(self.node.cluster_q)
        self.q_spin.valueChanged.connect(self.update_cluster_q)
        left_layout.addWidget(QLabel("Cluster Q(m):"))
        left_layout.addWidget(self.q_spin)

        self.b_spin = QDoubleSpinBox()
        self.b_spin.setMinimum(0.0)
        self.b_spin.setMaximum(2 * math.pi)
        self.b_spin.setValue(self.node.cluster_b)
        self.b_spin.valueChanged.connect(self.update_cluster_b)
        left_layout.addWidget(QLabel("Cluster B(rad):"))
        left_layout.addWidget(self.b_spin)
        right_layout = QVBoxLayout()
        mapping_header = QLabel("Button Assignments")
        right_layout.addWidget(mapping_header)
        mapping = {
            "forward/backward vel": self.node.get_parameter('lx').value,
            "rotation vel": self.node.get_parameter('az').value,
            "enable": self.node.get_parameter('en').value,
            "rover_sel": self.node.get_parameter('rover_sel').value,
            "mode": self.node.get_parameter('mode_sel').value,
            "angle_sel": self.node.get_parameter('angle_sel').value,
            "hardware/sim": self.node.get_parameter('hardware_sim_sel').value,
            "broadcast": self.node.get_parameter('broadcast').value,
        }
        self.mapping_labels = {}
        for key, value in mapping.items():
            label = QLabel(f"{key}: {value}")
            self.mapping_labels[key] = label
            right_layout.addWidget(label)

        main_layout.addLayout(left_layout)
        main_layout.addLayout(right_layout)

        central_widget.setLayout(main_layout)
        self.setCentralWidget(central_widget)

        self.update_timer = QTimer()
        self.update_timer.timeout.connect(self.update_status)
        self.update_timer.start(100)

    def update_status(self):
        self.status_labels["selected_rover"].setText(f"Selected Rover: {self.node.select}")
        self.status_labels["mode"].setText(f"Mode: {self.node.rover_modeC}")
        self.status_labels["selected_angle"].setText(f"Selected Angle: {self.node.selected_angle}")
        hardware_str = "HW" if self.node.hw_sel else "Sim"
        self.status_labels["hardware"].setText(f"Hardware: {hardware_str}")
        self.status_labels["cluster_x"].setText(f"Cluster Xc: {self.node.cluster_x}")
        self.status_labels["cluster_y"].setText(f"Cluster Yc: {self.node.cluster_y}")
        self.status_labels["cluster_t"].setText(f"Cluster Tc: {self.node.cluster_t}")
        self.status_labels["cluster_p"].setText(f"Cluster P: {self.node.cluster_p}")
        self.status_labels["cluster_q"].setText(f"Cluster Q: {self.node.cluster_q}")
        self.status_labels["cluster_b"].setText(f"Cluster B: {self.node.cluster_b}")

    def update_rover(self, value):
        self.node.select = value

    def update_mode(self, text):
        self.node.rover_modeC = text

    def update_angle(self, text):
        try:
            angle = int(text)
            self.node.selected_angle = angle
            angle_msg = Int16()
            angle_msg.data = angle
            self.node.pubsub.publish('/joy/angle_sel', angle_msg)
        except ValueError:
            pass

    def update_hw(self, checked):
        self.node.hw_sel = checked

    def update_cluster_x(self, value):
        self.node.cluster_x = value
        msg = Float32MultiArray()
        msg.data = [self.node.cluster_x, self.node.cluster_y, self.node.cluster_t]
        self.node.pubsub.publish('/cluster_desired', msg)
    
    def update_cluster_y(self, value):
        self.node.cluster_y = value
        msg = Float32MultiArray()
        msg.data = [self.node.cluster_x, self.node.cluster_y, self.node.cluster_t]
        self.node.pubsub.publish('/cluster_desired', msg)
    
    def update_cluster_t(self, value):
        self.node.cluster_t = value
        msg = Float32MultiArray()
        msg.data = [self.node.cluster_x, self.node.cluster_y, self.node.cluster_t]
        self.node.pubsub.publish('/cluster_desired', msg)

    def update_cluster_p(self, value):
        self.node.cluster_p = value
        msg = Float32MultiArray()
        msg.data = [self.node.cluster_p, self.node.cluster_q, self.node.cluster_b]
        self.node.pubsub.publish('/cluster_params', msg)

    def update_cluster_q(self, value):
        self.node.cluster_q = value
        msg = Float32MultiArray()
        msg.data = [self.node.cluster_p, self.node.cluster_q, self.node.cluster_b]
        self.node.pubsub.publish('/cluster_params', msg)

    def update_cluster_b(self, value):
        self.node.cluster_b = value
        msg = Float32MultiArray()
        msg.data = [self.node.cluster_p, self.node.cluster_q, self.node.cluster_b]
        self.node.pubsub.publish('/cluster_params', msg)

def main(args=None):
    rclpy.init(args=args)
    node = JoyCmd()

    app = QApplication(sys.argv)
    gui = StatusWindow(node)
    gui.show()

    ros_timer = QTimer()
    ros_timer.timeout.connect(lambda: rclpy.spin_once(node, timeout_sec=0.01))
    ros_timer.start(10)

    exit_code = app.exec()
    node.destroy_node()
    rclpy.shutdown()
    sys.exit(exit_code)


if __name__ == '__main__':
    main()
