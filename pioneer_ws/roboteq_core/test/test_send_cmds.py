import serial
import time

RETRIES = 6
READ_SLEEP = 0.001
PORT = "/dev/ttyACM0"
BAUDRATE = 115200

class Connect:
    def __init__(self, port: str, baudrate: int):
        self.port = port
        self.baudrate = baudrate

    def handle_request(self, cmd: str) -> str:
        resp = ""
        with serial.Serial(self.port, self.baudrate) as ser:
            for retry in range(RETRIES):
                ser.write(cmd.encode())
                time.sleep(READ_SLEEP)
                resp = ser.read_all().decode('utf-8')
                if resp != "":
                    return resp

        return resp

    def send_write_command(self, cmd: str) -> None:
        with serial.Serial(self.port, self.baudrate) as ser:
            ser.write(cmd.encode())


def main():
    connect = Connect(PORT, BAUDRATE)
    connect.send_write_command("!M {} {}_".format("4", "4"))

    for retry in range(RETRIES):
        resp = connect.handle_request("?SPE _")
        if "=" in resp:
            resp = resp[-3:]
            print(float(resp[0]), float(resp[2]))


if __name__ == '__main__':
    main()