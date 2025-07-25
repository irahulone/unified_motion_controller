import rclpy
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import Int32MultiArray, Float32


# Custom submodules
from .connect import Connect, handle_shell_request
from .publishers import battery, drive_unit, shell_user_ip
from .subscribers import move

BAUDRATE = 115200


platform_topics = {
    "pioneer": {
        "move": "{}/ch_vals",
        "encoder": "{}/du/encoder_val",
        "velocity": "{}/du/velocity"
    },
    "agbot": {
        "move": "{}/du{}/vel",
        "encoder": "{}/du{}/encoder_val",
        "velocity": "{}/du{}/velocity"
    }
}

def modify_user(resp_user) -> str:
    replace_characters = {
        '-': '_'
    }

    new = ""
    for c in resp_user:
        if replace_characters.get(c):
            c = replace_characters[c]
        new += c
    return new

def get_user() -> str:
    cmd = 'echo $USER'
    return modify_user(handle_shell_request(cmd=cmd))

def get_ports():
    cmd = 'ls /dev/tty* | grep "ACM\\|USB"'
    resp = handle_shell_request(cmd=cmd)
    return resp.split('\n')

def add_executor_nodes(executor):
    user = get_user()
    ports = get_ports()
    if len(ports) > 1:
        topics = platform_topics["agbot"]
        msg = "Agbot platform detected"
    else:
        topics = platform_topics["pioneer"]
        msg = "Pioneer platform detected"

    print(msg)

    count = 1
    for port in ports:
        conn = Connect(port, BAUDRATE)
        if len(ports) > 1:
            move_topic = topics["move"].format(user, count)
            encoder_topic = topics["encoder"].format(user, count)
            velocity_topic = topics["velocity"].format(user, count)
            battery_user = user + '_' + str(count)
            msg_type = Float32
        else:
            move_topic = topics["move"].format(user)
            encoder_topic = topics["encoder"].format(user)
            velocity_topic = topics["velocity"].format(user)
            battery_user = user
            msg_type = Int32MultiArray

        executor.add_node(move.Move(conn, move_topic, msg_type))
        executor.add_node(drive_unit.Publisher(conn, encoder_topic, velocity_topic))
        executor.add_node(battery.Publisher(conn, battery_user))

        count += 1

    executor.add_node(shell_user_ip.Publisher(user))


def main(args=None):
    rclpy.init(args=args)
    executor = MultiThreadedExecutor()
    add_executor_nodes(executor)
    executor.spin()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
