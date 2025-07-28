#!/usr/bin/env python3
import argparse
import rosbag2_py
import os
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
import matplotlib.pyplot as plt

ros_sqlite3 = ["foxy", "galactic", "humble"]

def read_ros2_bag(bag_path: str):
    reader = rosbag2_py.SequentialReader()
    ros_distro = os.getenv('ROS_DISTRO')
    if ros_distro in ros_sqlite3:
        storage_id='sqlite3'
    else:
        storage_id='mcap'
    
    storage_options = rosbag2_py.StorageOptions(
        uri=bag_path,
        storage_id=storage_id
    )
    
    converter_options = rosbag2_py.ConverterOptions('', '')
    
    reader.open(storage_options, converter_options)
    
    topics_and_types = reader.get_all_topics_and_types()
    
    # ex.) {'/some_topic': 'std_msgs/msg/String', '/another_topic': 'sensor_msgs/msg/Image', ...}
    type_dict = { t.name: t.type for t in topics_and_types }

    topic_messages = {t.name: [] for t in topics_and_types}
    
    print("== topics ==")
    for t_name, t_type in type_dict.items():
        print(f"{t_name}: {t_type}")
    print("================\n")
    
    while reader.has_next():
        topic_name, data, t_stamp = reader.read_next()
        
        msg_type_str = type_dict[topic_name]
        msg_type = get_message(msg_type_str)
        
        msg = deserialize_message(data, msg_type)
        topic_messages[topic_name].append((t_stamp, msg))
    
    x_c = []
    y_c = []
    p = []
    q = []
    B = []
    x_c_des = []
    y_c_des = []
    p_des = []
    q_des = []
    B_des = []
    ts = []
    for _ts, msg in topic_messages["/sim/cluster_info"]:
        ts.append(_ts)
        x_c.append(msg.cluster.data[0])
        y_c.append(msg.cluster.data[1])
        p.append(msg.cluster.data[6])
        q.append(msg.cluster.data[7])
        B.append(msg.cluster.data[8])
        x_c_des.append(msg.cluster_desired.data[0])
        y_c_des.append(msg.cluster_desired.data[1])
        p_des.append(msg.cluster_desired.data[6])
        q_des.append(msg.cluster_desired.data[7])
        B_des.append(msg.cluster_desired.data[8])
    ts0 = ts[0]
    ts = [(t - ts0)/(10**9) for t in ts]

    os.makedirs(bag_path+"/figures", exist_ok=True)
    fig = plt.figure()
    plt.plot(x_c_des, y_c_des, label="Cluster Desired")
    plt.plot(x_c, y_c, label="Cluster")

    plt.title("x_c vs y_c")
    plt.xlabel("X-axis")
    plt.ylabel("Y-axis")
    plt.legend()
    fig.savefig(f"{bag_path}/figures/cluster.png")
    plt.show()

    fig = plt.figure()
    plt.plot(ts, x_c_des, label="desired x_c")
    plt.plot(ts, x_c, label="actual x_c")

    plt.title("x_c vs t")
    plt.xlabel("t")
    plt.ylabel("x_c")
    plt.legend()
    fig.savefig(f"{bag_path}/figures/x_c.png")
    plt.show()

    fig = plt.figure()
    plt.plot(ts, y_c_des, label="desired y_c")
    plt.plot(ts, y_c, label="actual y_c")

    plt.title("y_c vs t")
    plt.xlabel("t")
    plt.ylabel("y_c")
    plt.legend()
    fig.savefig(f"{bag_path}/figures/y_c.png")
    plt.show()

    fig = plt.figure()
    plt.plot(ts, p_des, label="desired p")
    plt.plot(ts, p, label="actual p")

    plt.title("p vs t")
    plt.xlabel("t")
    plt.ylabel("p")
    plt.legend()
    fig.savefig(f"{bag_path}/figures/p.png")
    plt.show()

    fig = plt.figure()
    plt.plot(ts, q_des, label="desired q")
    plt.plot(ts, q, label="actual q")

    plt.title("q vs t")
    plt.xlabel("t")
    plt.ylabel("q")
    plt.legend()
    fig.savefig(f"{bag_path}/figures/q.png")
    plt.show()

    fig = plt.figure()
    plt.plot(ts, B_des, label="desired B")
    plt.plot(ts, B, label="actual B")

    plt.title("B vs t")
    plt.xlabel("t")
    plt.ylabel("B")
    plt.legend()
    fig.savefig(f"{bag_path}/figures/B.png")
    plt.show()


def main():
    parser = argparse.ArgumentParser(
        description="ROS 2 bag reader"
    )

    parser.add_argument(
        "bag_path", 
        help="ROS 2 bag path"
    )
    args = parser.parse_args()
    
    read_ros2_bag(args.bag_path)

if __name__ == "__main__":
    main()