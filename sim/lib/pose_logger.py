import sys
import rclpy
import json
import time
import csv
import threading
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

# PyQt6への変更点:
# 列挙型が Qt.AlignmentFlag, Qt.ItemFlag, Qt.TextFlag に分割されている点などに注意
from PyQt6.QtCore import (
    Qt, QSize, pyqtSignal, QObject
)
from PyQt6.QtWidgets import (
    QApplication, QWidget, QVBoxLayout, QTableWidget, QTableWidgetItem,
    QHeaderView
)
from PyQt6.QtGui import QFontMetrics
from threading import Thread, Event

from rosidl_runtime_py.utilities import get_message
from geometry_msgs.msg import Pose2D


class CSVLogger:
    """
    Pose2Dの書き込みをスレッドセーフに行うためのロガークラス。
    """
    def __init__(self, filename):
        self.filename = filename
        self.lock = threading.Lock()
        
        # CSVファイルを初期化（ヘッダ行を書き込む）: 
        # 既存ファイルを上書きする想定の場合は "w" 、追記したい場合は "a" に切り替えてください
        with open(self.filename, "w", newline="", encoding="utf-8") as f:
            writer = csv.writer(f)
            writer.writerow(["message_name", "timestamp", "x", "y", "theta"])

    def log_pose2d(self, topic_name, x, y, theta):
        """
        Pose2DメッセージをCSVに追記する。
        topic_name: トピック名 (ここでは"メッセージ名"として扱う)
        """
        timestamp = time.time()
        with self.lock:
            with open(self.filename, "a", newline="", encoding="utf-8") as f:
                writer = csv.writer(f)
                writer.writerow([topic_name, timestamp, x, y, theta])


class UpdateSignal(QObject):
    """
    他スレッドからのGUI更新を受け付けるためのSignal。
    topic_name(str), formatted_msg(str)を受け取ってスレッドセーフにGUI更新へ渡す。
    """
    new_message = pyqtSignal(str, str)


class ROS2Pose2DMonitor(Node):
    """
    geometry_msgs/msg/Pose2D 型のトピックのみを購読し、
    受信内容をCSVに保存するNode。
    """
    def __init__(self, update_signal, csv_logger):
        super().__init__('ros2_pose2d_monitor')
        self.update_signal = update_signal
        self.csv_logger = csv_logger
        self.subscribers = {}

        # 定期的にトピック一覧を更新して、Pose2D だけサブスクライブ
        self.create_timer(1.0, self.update_topics)

    def update_topics(self):
        topic_names_and_types = self.get_topic_names_and_types()
        for topic, types in topic_names_and_types:
            if topic not in self.subscribers and "geometry_msgs/msg/Pose2D" in types:
                msg_type = get_message("geometry_msgs/msg/Pose2D")
                self.create_pose2d_subscriber(topic, msg_type)

    def create_pose2d_subscriber(self, topic_name, msg_type):
        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )

        def callback(msg):
            # 受信したPose2DメッセージをJSON形式に整形してGUIへ渡す
            msg_dict = {
                "x": msg.x,
                "y": msg.y,
                "theta": msg.theta
            }
            formatted_msg = json.dumps(msg_dict, indent=2)
            self.update_signal.new_message.emit(topic_name, formatted_msg)

            # CSVに書き込み
            self.csv_logger.log_pose2d(topic_name, msg.x, msg.y, msg.theta)

        sub = self.create_subscription(
            msg_type, topic_name, callback, qos
        )
        self.subscribers[topic_name] = sub


class ROS2GUI(QWidget):
    """
    Pose2D型トピックを監視して、そのメッセージをGUIに表示するとともに
    CSVへ保存する簡易アプリケーション(PyQt6対応)。
    """
    def __init__(self):
        super().__init__()
        self.setWindowTitle("ROS2 Pose2D Monitor (Subscribe Only, CSV Logging) - PyQt6")
        self.setGeometry(100, 100, 800, 600)

        self.layout = QVBoxLayout()
        self.table = QTableWidget()
        self.table.setColumnCount(2)
        self.table.setHorizontalHeaderLabels(["Topic", "Latest Pose2D"])
        self.layout.addWidget(self.table)
        self.setLayout(self.layout)

        self.table.horizontalHeader().setSectionResizeMode(
            0, QHeaderView.ResizeMode.ResizeToContents)
        self.table.horizontalHeader().setSectionResizeMode(
            1, QHeaderView.ResizeMode.Stretch)
        self.table.setWordWrap(True)

        self.update_signal = UpdateSignal()
        self.update_signal.new_message.connect(self.update_table)

        # CSVロガーの初期化
        self.csv_logger = CSVLogger("pose2d_log.csv")

        # ROSノード起動
        self.node = ROS2Pose2DMonitor(self.update_signal, self.csv_logger)
        self.ros_thread_stop_event = Event()
        self.ros_thread = Thread(target=self.run_ros_spin, daemon=True)
        self.ros_thread.start()

    def run_ros_spin(self):
        rclpy.spin(self.node)

    def closeEvent(self, event):
        # ウィンドウを閉じる際にROSを停止
        self.node.destroy_node()
        rclpy.shutdown()
        self.ros_thread_stop_event.set()
        super().closeEvent(event)

    def update_table(self, topic, msg):
        row = self.get_row_for_topic(topic)
        self.table.setItem(row, 0, QTableWidgetItem(topic))

        item = QTableWidgetItem(msg)
        # PyQt6 では alignment などのenumは AlignmentFlag を使用
        item.setTextAlignment(Qt.AlignmentFlag.AlignTop)

        # アイテムを選択のみ可能、編集不可にする
        flags = Qt.ItemFlag.ItemIsEnabled | Qt.ItemFlag.ItemIsSelectable
        item.setFlags(flags)

        # QFontMetrics: boundingRect のシグネチャ用に TextFlag 参照を使う
        font_metrics = QFontMetrics(self.table.font())
        text_rect = font_metrics.boundingRect(
            0, 0,
            self.table.columnWidth(1),
            0,
            int(Qt.TextFlag.TextWordWrap),
            msg
        )
        item.setSizeHint(QSize(self.table.columnWidth(1), text_rect.height() + 10))

        self.table.setItem(row, 1, item)
        self.table.resizeRowsToContents()

    def get_row_for_topic(self, topic):
        for row in range(self.table.rowCount()):
            if self.table.item(row, 0) and self.table.item(row, 0).text() == topic:
                return row
        self.table.insertRow(self.table.rowCount())
        return self.table.rowCount() - 1


def main():
    rclpy.init()
    app = QApplication(sys.argv)
    gui = ROS2GUI()
    gui.show()
    exit_code = app.exec()
    sys.exit(exit_code)


if __name__ == '__main__':
    main()
