import logging

logger = logging.getLogger()
logger.setLevel(logging.WARNING)

RETRIES = 6


class Status:
    """
    @class: Query ROBOTEQ controller for certain stats
    """
    def __init__(self, connection):
        # ROBOTEQ commands SDC2160 controller
        self.read_cmds = {
            "actual_velocity": "?SPE_",
            "encoder": "?C_",
            "battery_voltage": "?V 2_"
        }
        self.connect = connection

    def read_battery_12v(self) -> float:
        """
        Query battery voltage and parse the list of strings for battery voltage
        :return: Volts in float
        """
        batt_voltage = 0
        read_timer = 0.01  # seconds, anything faster would make battery voltage readings unpredictable

        for retry in range(RETRIES):
            resp = self.connect.handle_serial_request(self.read_cmds["battery_voltage"], read_timer)
            resp = resp.split('\r')
            if len(resp) > 1 and len(resp[1]) > 1 and ("V=" in resp[1]):
                # Give me 132 in response list ['?V 2', 'V=132'] or ['V=132', '?V 2']
                try:
                    batt_voltage = float(resp[1][2:]) / 10
                except ValueError as e:
                    logging.warning(f"Retrying. Exception: {e}")

        return float(batt_voltage)


    def read_velocity(self) -> tuple:
        """
        response is given as: SPE=3:3 or ?SPE=3:3 or any other variation

        :return: (3.0, 3.0) as a tuple for left and right side wheel velocities
        """
        read_timer = 0.001  # seconds, anything faster would make reading velocities inconsistent
        vel_wheel1 = vel_wheel2 = 0
        vel_limit = 10000

        for retry in range(RETRIES):
            resp = self.connect.handle_serial_request(self.read_cmds["actual_velocity"], read_timer)
            if not resp:
                continue
            resp = resp.split('\r')
            if len(resp) > 1 and len(resp[1]) == 5 and ("SPE=" in resp[1]):
                try:
                    resp = resp[1][2:].split(":")
                    vel_wheel1 = int(resp[0])
                    vel_wheel2 = int(resp[1])
                except ValueError as e:
                    logging.warning(f"Retrying. Exception: {e}")
                else:
                    logging.warning(f"1: {vel_wheel1}, 2: {vel_wheel2}")
                    if vel_wheel1 >= vel_limit or vel_wheel1 <= -vel_limit:
                        vel_wheel1 = 0
                    if vel_wheel2 >= vel_limit or vel_wheel2 <= -vel_limit:
                        vel_wheel2 = 0
                logging.warning(f"1: {vel_wheel1}, 2: {vel_wheel2}")
                break

        return vel_wheel1, vel_wheel2

    def read_encoder(self) -> tuple:
        """
        Read the encoder value from the roboteq
        """
        encoder1 = encoder2 = 0
        read_timer = 0.001
        for retry in range(RETRIES):
            resp = self.connect.handle_serial_request(self.read_cmds["encoder"], read_timer)
            if not resp:
                continue
            resp = resp.split('\r')
            if len(resp) > 1 and ("C=" in resp[1]):
                resp = resp[1][2:].split(":")
                try:
                    encoder1 = int(resp[0])
                    encoder2 = int(resp[1])
                    break
                except (ValueError, IndexError) as e:
                    logging.warning(f"Retrying. Exception: {e}")

        return encoder1, encoder2