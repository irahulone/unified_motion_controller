import matplotlib
matplotlib.use('Qt5Agg')  # Qt用のバックエンドを使用

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
import pyproj
import matplotlib.pyplot as plt
from PyQt6.QtWidgets import QApplication, QMainWindow, QVBoxLayout, QWidget
from PyQt6.QtCore import QTimer, QThread, pyqtSignal
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
import sys

class GPSPlotter(Node):
    def __init__(self):
        super().__init__('gps_plotter')
        self.subscription_dict = {
            '/p2': self.create_subscription(NavSatFix, '/p2/gps1', self.listener_callback_factory('/p2'), 10),
            '/p3': self.create_subscription(NavSatFix, '/p3/gps1', self.listener_callback_factory('/p3'), 10),
            '/p4': self.create_subscription(NavSatFix, '/p4/gps1', self.listener_callback_factory('/p4'), 10)
        }
        self.proj = pyproj.Proj(proj='utm', zone=54, ellps='WGS84', south=False)
        self.trackers = {}  # 複数のトピックを個別に管理
        self.origin_x, self.origin_y = None, None  # 共通の基準点
        self.init_plot()

    def listener_callback_factory(self, topic_name):
        def listener_callback(msg):
            x, y = self.proj(msg.longitude, msg.latitude)
            status = msg.status.status
            if status == 0:
                return 
            
            if self.origin_x is None or self.origin_y is None:
                self.origin_x, self.origin_y = x, y  # 最初に受信した座標を共通の原点に設定
            
            if topic_name not in self.trackers:
                self.trackers[topic_name] = {'x_data': [], 'y_data': [], 'plot_line': None}
                self.trackers[topic_name]['plot_line'], = self.ax.plot([], [], marker='o', linestyle='-', label=topic_name)
                self.ax.legend()
            
            tracker = self.trackers[topic_name]
            rel_x = x - self.origin_x  # 東が+ 西が-
            rel_y = y - self.origin_y  # 北が+ 南が-
            
            tracker['x_data'].append(rel_x)
            tracker['y_data'].append(rel_y)
            print(f"Received GPS ({topic_name}): lat={msg.latitude}, lon={msg.longitude} → x={rel_x:.2f} m, y={rel_y:.2f} m")
        return listener_callback

    def init_plot(self):
        self.fig, self.ax = plt.subplots()
        self.canvas = FigureCanvas(self.fig)
        self.ax.set_title("Real-time GPS Plot")
        self.ax.set_xlabel("X (meters)")
        self.ax.set_ylabel("Y (meters)")

    def update_plot(self):
        for tracker in self.trackers.values():
            tracker['plot_line'].set_xdata(tracker['x_data'])
            tracker['plot_line'].set_ydata(tracker['y_data'])
        self.ax.relim()
        self.ax.autoscale_view()
        self.canvas.draw()

class ROS2Thread(QThread):
    def __init__(self, node):
        super().__init__()
        self.node = node

    def run(self):
        rclpy.spin(self.node)

class GPSPlotGUI(QMainWindow):
    def __init__(self, gps_plotter):
        super().__init__()
        self.setWindowTitle("GPS Plotter")
        self.setGeometry(100, 100, 600, 400)
        self.central_widget = QWidget()
        self.setCentralWidget(self.central_widget)
        layout = QVBoxLayout()
        layout.addWidget(gps_plotter.canvas)
        self.central_widget.setLayout(layout)
        self.gps_plotter = gps_plotter
        self.timer = QTimer()
        self.timer.timeout.connect(self.update)
        self.timer.start(100)

    def update(self):
        self.gps_plotter.update_plot()

def main():
    rclpy.init()
    app = QApplication(sys.argv)
    gps_plotter = GPSPlotter()
    ros_thread = ROS2Thread(gps_plotter)
    ros_thread.start()
    gui = GPSPlotGUI(gps_plotter)
    gui.show()
    sys.exit(app.exec())

if __name__ == '__main__':
    main()
