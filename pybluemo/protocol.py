import random
from bgapi.module import BlueGigaClient, GATTService, GATTCharacteristic, ProcedureManager
from abc import ABCMeta, abstractmethod
import logging
import sys
import time

term = logging.StreamHandler(sys.stdout)
formatter = logging.Formatter('%(asctime)s - %(name)s - %(levelname)s - %(message)s')
term.setFormatter(formatter)
logger = logging.getLogger("Bluemo")
logger.addHandler(term)
logger.setLevel(logging.INFO)
#logging.basicConfig(filename='battery_voltage.log',level=logging.INFO)

BLUEMO_SRV_UUID = "\x15\x52\x00\x01\x9C\x32\xC9\x81\x43\x4E\xE1\xB4\xD5\xB4\x41\x3F"[::-1]
BLUEMO_CHR_UUID = "\x15\x52\x00\x02\x9C\x32\xC9\x81\x43\x4E\xE1\xB4\xD5\xB4\x41\x3F"[::-1]

SYNC_1 = "\xAA"
SYNC_2 = "\x55"

BLUEMO_CMD_CODES = {"CMD_EXEC_ERR": "\x00",
                    "LOOPBACK":     "\x01",
                    "PWR_CTRL":  "\x02",
                    "SET_WS2811":     "\x03",
                    "DIFF_ENC_WS2811": "\x04",
                    "SERVO_CFG": "\x05",
                    "I2C_XFER": "\x06",
                    "GPIO_CFG":     "\x07",
                    "UART_CONFIG":  "\x08",
                    "UART_TX":      "\x09",
                    "HALF_DUPLEX":  "\x0A",
                    "ADV_CONFIG":   "\x0B" }

BLUEMO_CMD_NAMES = {}
for key, value in BLUEMO_CMD_CODES.iteritems():
    BLUEMO_CMD_NAMES[value] = key

I2C_BAUD = {"100kHz": 0, "250kHz": 1, "400kHz": 2}

SPI_BAUD_E = {"125kbps": 0, "250kbps": 1, "500kbps": 2, "1Mbps": 3, "2Mbps": 4, "4Mbps": 5, "8Mbps": 6}

UART_BAUD_E = {"1200bps": 0, "2400bps": 1, "4800bps": 2, "9600bps": 3, "14400bps": 4, "19200bps": 5, "28800bps": 6, "38400bps": 7,
               "57600bps": 8, "76800bps": 9, "115200bps": 10, "230400bps": 11, "250000bps": 12, "460800bps": 13, "921600bps": 14, "1Mbps": 15 }

PORT_PIN_E = {"Port A Pin 0": 16, "Port A Pin 1": 17, "Port A Pin 2": 18, "Port A Pin 3": 19, "Port A Pin 4": 20, "Port A Pin 5": 21,
              "Port B Pin 0": 32, "Port B Pin 1": 33, "Port B Pin 2": 34, "Port B Pin 3": 35, "Port B Pin 4": 36, "Port B Pin 5": 37,
              "Port C Pin 0": 48, "Port C Pin 1": 49, "Port C Pin 2": 50, "Port C Pin 3": 51, "Port C Pin 4": 52, "Port C Pin 5": 53,
              "Port D Pin 0": 64, "Port D Pin 1": 65, "Port D Pin 2": 66, "Port D Pin 3": 67, "Port D Pin 4": 68, "Port D Pin 5": 69,
              "Debug Pin 0": 80, "Debug Pin 1": 81, "3V3 Enable": 82, "Main Power Enable": 83, "Charge Enable": 84, "Port Invalid": 255}

PIN_CFG_E = {"PIN_INPUT_NOPULL": 0, "PIN_INPUT_PULLUP": 1, "PIN_INPUT_PULLDOWN": 2, "PIN_OUTPUT_HIGH": 3, "PIN_OUTPUT_LOW": 4,
                  "PIN_OUTPUT_HIGH_OPEN": 4, "PIN_OUTPUT_LOW_OPEN": 5, "PIN_RESET_DEFAULT": 255 }

TX_POWER_E = {"-30dBm": -30, "-20dBm": -20, "-16dBm": -16, "-12dBm": -12, "-8dBm": -8, "-4dBm": -4, "0dBm": 0, "4dBm": 4 }


def to_little_endian(value, length):
    return_string = ""
    for i in range(length):
        return_string += chr((value >> (8*i)) & 0xFF)
    return return_string


def from_little_endian(str_data):
    result = 0
    for i in range(len(str_data)):
        result += ord(str_data[i]) << (8*i)
    return result


class AbstractBaseBluemo(ProcedureManager):
    __metaclass__ = ABCMeta
    def __init__(self, mesh):
        super(AbstractBaseBluemo, self).__init__()
        self._mesh_defn = mesh
        self.buffer = ""

    @abstractmethod
    def serial_tx(self, data): raise NotImplementedError()

    @abstractmethod
    def serial_rx(self, data): raise NotImplementedError()

    def serial_protocol(self):
        while len(self.buffer) > 2 and self.buffer[0] != SYNC_1:
            self.buffer = self.buffer[1:]
        if self.buffer[1] != SYNC_2:
            self.buffer = self.buffer[2:]
            self.serial_protocol()
            return
        length = from_little_endian(self.buffer[2:4])
        if len(self.buffer) - 5 >= length:
            message = self.buffer[5:5+length]
            rx_checksum = ord(self.buffer[4])
            self.buffer = self.buffer[5+length:]
            calc_checksum = sum([ord(i) for i in message]) & 0xFF
            if rx_checksum == calc_checksum:
                self.cmd_handler(message)
            else:
                logger.log(logging.INFO, "Checksum mismatch: %d != %d" % (rx_checksum, calc_checksum))

    def cmd_handler(self, data):
        logger.log(logging.DEBUG, "Command Received: " + "-".join(["%02X" % ord(i) for i in data]))
        if data[0] == BLUEMO_CMD_CODES["PWR_CTRL"]:
            logger.log(logging.INFO, "%d Power Control Response - 3.3V:%d - Main:%d - 3.3VDIS:%d - MainDIS:%d - CHRG_EN:%d - HEAT:%d - CHRG_RATE:%d - BATT_VOLT:%d - EXTERN_VOLT:%d" %
                       (time.time(), ord(data[2]), ord(data[3]), ord(data[4]), ord(data[5]), ord(data[6]), ord(data[7]), ord(data[8]), from_little_endian(data[9:11]), from_little_endian(data[11:])))

    def send_command(self, cmd, payload, ack):
        msg = SYNC_1 + SYNC_2 + to_little_endian(len(payload) + 2, 2)
        checksum = sum([ord(i) for i in (cmd + "\x00" + payload)])
        msg += chr(checksum & 0xFF) + cmd + "\x00" + payload
        self.serial_tx(msg)

    def loopback(self, data):
        self.send_command(BLUEMO_CMD_CODES["LOOPBACK"], payload=data, ack=None)

    def i2c_transaction(self, scl, sda, address, read_length, write_payload, periodicity=0, baud=2):
        data = chr(periodicity) + chr(baud) + chr(scl) + chr(sda) + chr(address)
        data += to_little_endian(read_length, 2) + to_little_endian(len(write_payload), 2)
        data += write_payload
        self.send_command(BLUEMO_CMD_CODES["I2C_TRANSACTION"], data, None)

    def servo_cfg(self, port_pin, pulse_width):
        data = chr(port_pin) + to_little_endian(pulse_width, 2)
        self.send_command(BLUEMO_CMD_CODES["SERVO_CFG"], data, None)

    def gpio_cfg(self, port_pin, config):
        data = chr(port_pin) + chr(config)
        self.send_command(BLUEMO_CMD_CODES["GPIO_CFG"], data, None)

    def read_gpio_in(self, periodicity=0):
        data = chr(periodicity)
        self.send_command(BLUEMO_CMD_CODES["READ_GPIO_IN"], data, None)

    def uart_config(self, speed, tx_pin, rx_pin, rts_pin=0xFF, cts_pin=0xFF):
        data = chr(speed) + chr(tx_pin) + chr(rx_pin) + chr(rts_pin) + chr(cts_pin)
        self.send_command(BLUEMO_CMD_CODES["UART_CONFIG"], data, None)

    def uart_tx(self, payload):
        self.send_command(BLUEMO_CMD_CODES["UART_TX"], payload, None)

    def half_duplex(self, handle, pin, baud, parity, listen, repeat, payload):
        data = chr(handle) + chr(pin) + chr(baud) + chr(parity) + chr(listen) + chr(repeat) + payload
        self.send_command(BLUEMO_CMD_CODES["HALF_DUPLEX"], data, None)

    def set_ws2811(self, pin, length=1, rotate_speed=0, dim_speed=0, color_size=3, rgb_values="\x00\x00\x00"):
        data = chr(pin) + to_little_endian(length * color_size, 2) + chr(color_size) + chr(rotate_speed) + chr(dim_speed) + rgb_values
        self.send_command(BLUEMO_CMD_CODES["SET_WS2811"], data, None)

    def diff_enc_ws2811(self, pin, offset, length, color):
        data = chr(pin) + chr(offset & 0xFF) + (chr(length & 0xFF))
        data += chr(((offset >> 4) & 0xF0) | ((length >> 8) & 0x0F))
        data += color
        self.send_command(BLUEMO_CMD_CODES["DIFF_ENC_WS2811"], data, None)

    def adv_config(self, adv_mode, adv_interval, tx_power, persist_config, device_name):
        data = chr(adv_mode) + to_little_endian(adv_interval, 2) + chr(tx_power) + chr(0) + device_name
        self.send_command(BLUEMO_CMD_CODES["ADV_CONFIG"], data, None)

    def pwr_ctrl(self, pwr_3v3_enable=0, pwr_3v3_disable=0, pwr_main_enable=0, pwr_main_disable=0,
                 pwr_3v3_off_disconnect_enable=0, pwr_3v3_off_disconnect_disable=0,
                 pwr_main_off_disconnect_enable=0, pwr_main_off_disconnect_disable=0,
                 chrg_enable=0, chrg_disable=0, chrg_rate=0, overheat_protect_enable=0, overheat_protect_disable=0):
        data = chr(pwr_3v3_enable) + chr(pwr_3v3_disable) + chr(pwr_main_enable) + chr(pwr_main_disable) + \
            chr(pwr_3v3_off_disconnect_enable) + chr(pwr_3v3_off_disconnect_disable) + chr(pwr_main_off_disconnect_enable) + \
            chr(pwr_main_off_disconnect_disable) + chr(chrg_enable) + chr(chrg_disable) + chr(chrg_rate) + \
            chr(overheat_protect_enable) + chr(overheat_protect_disable)
        self.send_command(BLUEMO_CMD_CODES["PWR_CTRL"], data, None)


class BluemoBLE(AbstractBaseBluemo):
    def __init__(self, handle, wr_wo_resp, mesh="\x00"):
        super(BluemoBLE, self).__init__(mesh=mesh)
        self.serial_char_handle = handle
        self.write_wo_response = wr_wo_resp
        self.running_tx_count = 0
        self.running_rx_count = 0

    def serial_tx(self, data):
        self.running_tx_count += len(data)
        #print("Running TX count: %d" % (self.running_tx_count))
        full_packets = len(data) / 20
        for i in range(full_packets):
            self.write_wo_response(self.serial_char_handle, data[20*i:20*(i+1)], attempts=3)
            time.sleep(0.01)
        if (len(data) % 20) > 0:
            self.write_wo_response(self.serial_char_handle, data[20*full_packets:], attempts=3)

    def serial_rx(self, data):
        self.running_rx_count += len(data)
        #print("Running RX count: %d" % (self.running_rx_count))
        self.buffer += data
        self.serial_protocol()


class BluemoUART(AbstractBaseBluemo):
    def __init__(self, port, baud, timeout, mesh="\x00"):
        super(BluemoUART, self).__init__(mesh=mesh)
        self._serial = None  # TODO: Open serial comms

    def serial_tx(self, data):
        pass

    def serial_rx(self, data):
        pass


class BluemoBlueGigaClient(BlueGigaClient):
    def connect_bluemo(self, name, scan_timeout=2):
        connection = self.connect_by_adv_data(name, scan_timeout=scan_timeout)
        connection.read_by_group_type(GATTService.PRIMARY_SERVICE_UUID)
        for service in connection.get_services():
            if service.uuid == BLUEMO_SRV_UUID:
                connection.find_information(service=service)
                connection.read_by_type(service=service, type=GATTCharacteristic.CHARACTERISTIC_UUID)
                connection.read_by_type(service=service, type=GATTCharacteristic.CLIENT_CHARACTERISTIC_CONFIG)
        for characteristic in connection.get_characteristics():
            if characteristic.has_notify():
                connection.characteristic_subscription(characteristic=characteristic, indicate=False, notify=True)
        serial_chr_handle = connection.get_handles_by_uuid(BLUEMO_CHR_UUID)[0]
        bluemo = BluemoBLE(serial_chr_handle, connection.wr_noresp_by_handle)
        connection.assign_attrclient_value_callback(serial_chr_handle, bluemo.serial_rx)
        return bluemo


def main():
    client = BluemoBlueGigaClient(port="COM3")
    client.pipe_logs_to_terminal(level=logging.INFO)
    client.reset_ble_state()
    bluemo = client.connect_bluemo(name="Bluemo")
    time.sleep(0.1)
    bluemo.pwr_ctrl(pwr_3v3_enable=1, pwr_main_enable=1, pwr_3v3_off_disconnect_disable=1, pwr_main_off_disconnect_disable=1)
    bluemo.set_ws2811(PORT_PIN_E["Port 1 Pin 0"], length=300, rgb_values=30*"\x10\x10\x10")
    time.sleep(0.1)
    bluemo.diff_enc_ws2811(PORT_PIN_E["Port 1 Pin 0"], offset=30, length=10, color="\x20\x00\x00")
    bluemo.diff_enc_ws2811(PORT_PIN_E["Port 1 Pin 0"], offset=100, length=50, color="\x00\x30\x00")
    bluemo.diff_enc_ws2811(PORT_PIN_E["Port 1 Pin 0"], offset=270, length=10, color="\x00\x00\x40")
    time.sleep(10)
    client.disconnect(0)

'''
    for j in range(10):
        #bluemo.set_ws2811(PORT_PIN_E["Port 2 Pin 1"], length=10, rotate_speed=1, dim_speed=10, rgb_values="".join([(chr(random.randint(0,255)) + chr(random.randint(0,255)) + chr(random.randint(0,255))) for i in range(300)]))
        bluemo.set_ws2811(PORT_PIN_E["Port 2 Pin 0"], length=24, rotate_speed=1, dim_speed=5, rgb_values="".join(["\xFF\x00\x00" for i in range(10)]))
        time.sleep(1)
        bluemo.set_ws2811(PORT_PIN_E["Port 2 Pin 0"], length=24, rotate_speed=1, dim_speed=5, rgb_values="".join(["\x00\xFF\x00" for i in range(10)]))
        time.sleep(1)
        bluemo.set_ws2811(PORT_PIN_E["Port 2 Pin 0"], length=24, rotate_speed=1, dim_speed=5, rgb_values="".join(["\x00\x00\xFF" for i in range(10)]))
        time.sleep(1)

    time.sleep(1)
    client.disconnect(0)
'''

if __name__ == "__main__":
    main()
    time.sleep(1)

