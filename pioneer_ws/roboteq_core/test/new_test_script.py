import time
import logging

from subprocess import Popen, PIPE

logging.basicConfig(level=logging.INFO)
logging.getLogger(__name__)


RETRIES = 6
READ_SLEEP = 0.001  # 0.0001 seconds induces errors
RETRY_SLEEP = READ_SLEEP


def handle_shell_request(cmd: str) -> str:
    """
    cmd needs to be a shell command that you type into a terminal
    or else there will be strange errors
    :return: decoded message as a string
    """
    with Popen(f'{cmd}', stdout=PIPE, shell=True) as proc:
        for retry in range(RETRIES):
            try:
                stdout, errs = proc.communicate(timeout=5)
            except Exception as e:
                logging.error(f"Exception: {e}")
            else:
                if errs:
                    logging.error(f"Shell error: {errs}")
                else:
                    return stdout.decode('utf-8', "ignore").strip('\n')
    return ""


class Connect:
    """
    @class: Connect to serial port and read and write commands or run commands through shell
    """
    def __init__(self, port: str, baudrate: int):
        self.port = port
        self.baudrate = baudrate

    def handle_serial_request(self, cmd: str, read_timer: float) -> str:
        """
        Write through serial port and read the entire line

        :param cmd: command to send
        :param read_timer: the time in which to sleep before reading serial port
        :return: string response from the serial port
        """
        resp = ""
        with serial.Serial(self.port, self.baudrate) as serial_handler:
            for retry in range(RETRIES):
                try:
                    serial_handler.write(cmd.encode())
                    time.sleep(read_timer)
                    resp = serial_handler.read_all().decode('utf-8', "ignore")
                    if resp != "":
                        return resp
                except serial.serialutil.SerialException as e:
                    time.sleep(RETRY_SLEEP)
                    pass

        return resp

    def send_write_command(self, cmd: str) -> None:
        """
        :param cmd: command to send through the serial port
        :return: None
        """
        with serial.Serial(self.port, self.baudrate) as serial_handler:
            serial_handler.write(cmd.encode())


cmd = 'ls /dev/tty* | grep "ACM|print"'
print(type(handle_shell_request(cmd=cmd)))

