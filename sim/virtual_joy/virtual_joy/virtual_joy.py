import sys
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from PyQt6.QtWidgets import QApplication, QWidget, QVBoxLayout, QHBoxLayout, QPushButton, QSlider, QLabel
from PyQt6.QtCore import Qt, QTimer


FREQ = 10

class VirtualXbox(Node, QWidget):
    def __init__(self):
        Node.__init__(self, "virtual_xbox")
        QWidget.__init__(self)

        self.joy_pub = self.create_publisher(Joy, "joy", 10)
        self.setWindowTitle("ROS2 Virtual Xbox Controller")
        self.setGeometry(100, 100, 400, 600)

        layout = QVBoxLayout()

        self.sticks = {
            "Left X": QSlider(Qt.Orientation.Horizontal),
            "Left Y": QSlider(Qt.Orientation.Vertical),
            "Right X": QSlider(Qt.Orientation.Horizontal),
            "Right Y": QSlider(Qt.Orientation.Vertical),
            "LT": QSlider(Qt.Orientation.Horizontal),
            "RT": QSlider(Qt.Orientation.Horizontal),
        }

        for key, slider in self.sticks.items():
            slider.setRange(-100, 100)
            slider.setValue(0)
            #slider.valueChanged.connect()
            layout.addWidget(QLabel(key))
            layout.addWidget(slider)

        reset_button = QPushButton("Reset Sticks")
        reset_button.clicked.connect(self.reset_sticks)
        layout.addWidget(reset_button)

        self.buttons = []
        button_names = [
            "A", "B", "X", "Y", "LB", "RB", "Back", "Start", "Xbox", "L Stick", "R Stick"
        ]
        button_layout = QHBoxLayout()
        for i, name in enumerate(button_names):
            btn = QPushButton(name)
            btn.setCheckable(True)
            btn.clicked.connect(lambda _, idx=i: self.set_button(idx))
            self.buttons.append(btn)
            button_layout.addWidget(btn)
        layout.addLayout(button_layout)

        # D-Pad
        self.dpad = {
            "Up": QPushButton("↑"),
            "Down": QPushButton("↓"),
            "Left": QPushButton("←"),
            "Right": QPushButton("→"),
        }
        dpad_layout = QHBoxLayout()
        for key, btn in self.dpad.items():
            btn.setCheckable(True)
            btn.clicked.connect(lambda _, key=key: self.set_dpad(key))
            dpad_layout.addWidget(btn)
        layout.addLayout(dpad_layout)

        self.setLayout(layout)
        self.axes = [0.0] * 8
        self.buttons_state = [0] * len(button_names)

        self.timer = QTimer()
        self.timer.timeout.connect(self.publish_joy)
        self.timer.start(int(1000//FREQ))

    def reset_sticks(self):
        for slider in self.sticks.values():
            slider.setValue(0)

    def set_button(self, index):
        self.buttons_state[index] = 1 if self.buttons[index].isChecked() else 0

    def set_dpad(self, key):
        dpad_map = {"Up": 6, "Down": 6, "Left": 7, "Right": 7}
        values = {"Up": 1.0, "Down": -1.0, "Left": -1.0, "Right": 1.0}

        self.axes[dpad_map[key]] = values[key] if self.dpad[key].isChecked() else 0.0

    def publish_joy(self):
        self.axes[0] = self.sticks["Left X"].value() / 100.0
        self.axes[1] = self.sticks["Left Y"].value() / 100.0
        self.axes[2] = self.sticks["LT"].value() / 100.0
        self.axes[3] = self.sticks["Right X"].value() / 100.0
        self.axes[4] = self.sticks["Right Y"].value() / 100.0
        self.axes[5] = self.sticks["RT"].value() / 100.0

        msg = Joy()
        msg.axes = self.axes
        msg.buttons = self.buttons_state
        self.joy_pub.publish(msg)

def main():
    rclpy.init()
    app = QApplication(sys.argv)
    node = VirtualXbox()
    node.show()
    app.exec()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
