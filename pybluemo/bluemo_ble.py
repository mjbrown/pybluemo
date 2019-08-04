import logging
import sys
from abc import ABCMeta, abstractmethod
import time
import thread
from threading import Semaphore

from bgapi.module import BlueGigaClient, GATTService, GATTCharacteristic
from bgapi.module import BlueGigaServer, ProcedureManager, BLEScanResponse
from bgapi.cmd_def import gap_discover_mode, connection_status_mask

from bluemo_msg import *

term = logging.StreamHandler(sys.stdout)
formatter = logging.Formatter('%(asctime)s - %(name)s - %(levelname)s - %(message)s')
term.setFormatter(formatter)
logger = logging.getLogger("YASP")
logger.addHandler(term)
logger.setLevel(logging.INFO)

#error_handler = logging.FileHandler("error_log.txt")
#error_logger = logging.getLogger("YASP")
#error_logger.addHandler(error_handler)
#error_logger.setLevel(logging.ERROR)

YASP_BLE_MTU = 20
YASP_SRV_UUID = b"\x3F\x41\xB4\xD5\xB4\xE1\x4E\x43\x81\xC9\x32\x9C\x01\x00\x52\x15"
YASP_CHR_UUID = b"\x3F\x41\xB4\xD5\xB4\xE1\x4E\x43\x81\xC9\x32\x9C\x02\x00\x52\x15"


class AbstractBaseYasp(ProcedureManager):
    __metaclass__ = ABCMeta

    def __init__(self):
        super(AbstractBaseYasp, self).__init__()
        self.buffer = b""
        self.cached_response = None
        self.cmd_callbacks = {}
        self.def_callbacks = {}

    @abstractmethod
    def serial_tx(self, data): raise NotImplementedError()

    def serial_rx(self, data):
        logger.log(logging.DEBUG, "<=" + "".join(["%02X" % ord(j) for j in data]))
        self.buffer += data
        self.rx_processing()

    def rx_processing(self):
        if len(self.buffer) > 1:
            hdr_len = 2
            length = ord(self.buffer[0])
            if length > 127:
                if len(self.buffer) > hdr_len:
                    length = (ord(self.buffer[0]) & 0x7F) + ((ord(self.buffer[1]) & 0xF) << 7)
                    if self.buffer[1] > 0xF:
                        logger.error("RX Overrun, packets dropped.")
                    hdr_len += 1
                else:
                    return
            if len(self.buffer) >= hdr_len + length:
                cmd = ord(self.buffer[hdr_len-1])
                message = self.buffer[hdr_len:hdr_len+length]
                thread.start_new_thread(self.cmd_handler, (cmd, message))
                self.buffer = self.buffer[hdr_len+length:]
                self.rx_processing()

    def send_command(self, callback, msg_defn=MessageDefinition()):
        payload = msg_defn.cmd_msg()
        if len(payload) > 0x7F:
            msg = struct.pack("<BBB%ds" % len(payload), (len(payload) & 0x7F) | 0x80, len(payload) >> 7, msg_defn.get_command_code(), payload)
        else:
            msg = struct.pack("<BB%ds" % len(payload), len(payload), msg_defn.get_command_code(), payload)
        if callback is not None:
            if msg_defn.get_response_code() not in self.cmd_callbacks:
                self.cmd_callbacks[msg_defn.get_response_code()] = []
            self.cmd_callbacks[msg_defn.get_response_code()].append(callback)
            self.serial_tx(msg)
        elif msg_defn.get_response_code() in self.def_callbacks:
            self.serial_tx(msg)
        else:
            with self.procedure_call(msg_defn.get_response_code(), timeout=1):
                self.serial_tx(msg)
            return self.cached_response

    def send_response(self, msg_defn=MessageDefinition()):
        payload = msg_defn.rsp_msg()
        msg = struct.pack("<HB%ds" % len(payload), len(payload), msg_defn.get_response_code(), payload)
        self.serial_tx(msg)

    def set_default_msg_callback(self, code, callback):
        self.def_callbacks[code] = callback

    @abstractmethod
    def cmd_handler(self, cmd, payload):
        pass


class YaspClient(AbstractBaseYasp):
    def __init__(self):
        super(YaspClient, self).__init__()
        for key, value in MSG_CLASS_BY_RSP_CODE.iteritems():
            self.cmd_callbacks[key] = []
        self.serial_char_handle = None
        self.write_wo_response = None

    def set_interface(self, handle, wr_wo_resp):
        self.serial_char_handle = handle
        self.write_wo_response = wr_wo_resp

    def serial_tx(self, data):
        if not self.serial_char_handle:
            raise RuntimeError()
        if not self.write_wo_response:
            raise RuntimeError()
        full_packets = int(len(data) / YASP_BLE_MTU)
        for i in range(full_packets):
            logger.log(logging.DEBUG, "=>"+"".join(["%02X" % ord(j) for j in data[YASP_BLE_MTU*i:YASP_BLE_MTU*(i+1)]]))
            self.write_wo_response(self.serial_char_handle, data[YASP_BLE_MTU*i:YASP_BLE_MTU*(i+1)], attempts=3)
        if (len(data) % YASP_BLE_MTU) > 0:
            logger.log(logging.DEBUG, "=>" + "".join(["%02X" % ord(j) for j in data[YASP_BLE_MTU*full_packets:]]))
            self.write_wo_response(self.serial_char_handle, data[YASP_BLE_MTU*full_packets:], attempts=3)

    def cmd_handler(self, cmd, payload):
        #logger.log(logging.INFO, "Response received: %02x - " % cmd + "".join(["%02X" % ord(j) for j in payload]))
        if cmd not in MSG_CLASS_BY_RSP_CODE:
            raise RuntimeError()
        response = MSG_CLASS_BY_RSP_CODE[cmd].rsp_recv(payload)
        if len(self.cmd_callbacks[cmd]) == 0:       # Callback not specified
            if cmd in self.def_callbacks:           # Default callback specified
                self.def_callbacks[cmd](response)
            else:
                self.cached_response = response
                self.procedure_complete(cmd)
        else:
            callback = self.cmd_callbacks[cmd].pop()
            callback(response)


class YaspServer(AbstractBaseYasp):
    def __init__(self):
        super(YaspServer, self).__init__()
        self.serial_char_handle = None
        self.write_attribute = None

    def set_interface(self, handle, write_attr):
        self.serial_char_handle = handle
        self.write_attribute = write_attr

    def serial_tx(self, data):
        if not self.serial_char_handle:
            raise RuntimeError()
        if not self.write_attribute:
            raise RuntimeError()
        full_packets = int(len(data) / YASP_BLE_MTU)
        for i in range(full_packets):
            chunk = data[YASP_BLE_MTU*i:YASP_BLE_MTU*(i+1)]
            logger.log(logging.DEBUG, "=>" + "".join(["%02X" % ord(j) for j in chunk]))
            self.write_attribute(self.serial_char_handle, offset=0, value=chunk, timeout=1)
        if (len(data) % YASP_BLE_MTU) > 0:
            chunk = data[YASP_BLE_MTU*full_packets:]
            logger.log(logging.DEBUG, "=>" + "".join(["%02X" % ord(j) for j in chunk]))


class YaspBlueGigaClient(BlueGigaClient):
    def __init__(self, port, baud=115200, timeout=1):
        super(YaspBlueGigaClient, self).__init__(port, baud, timeout)
        self.scan_filter_name = None
        self.scan_filter_service = None
        self.scan_filter_result = None

    def connect_by_name(self, name, yasp_client, conn_interval_min=0x20,
                        conn_interval_max=0x30, connection_timeout=100, latency=0, scan_timeout=1):
        self.scan_filter_name = name
        now = start = time.time()
        self._api.ble_cmd_gap_discover(mode=gap_discover_mode['gap_discover_observation'])
        while now < start + scan_timeout and self.scan_filter_result is None:
            time.sleep(scan_timeout - (now - start))
            now = time.time()
        self._api.ble_cmd_gap_end_procedure()
        if self.scan_filter_result is not None:
            logger.log(logging.INFO, "Intiating connection to %s" % ":".join(["%02X" % ord(i) for i in self.scan_filter_result.sender[::-1]]))
            connection = self.connect(self.scan_filter_result, scan_timeout, conn_interval_min,
                                      conn_interval_max, connection_timeout, latency)
            logger.log(logging.INFO, "Connected.")
            connection.read_by_group_type(GATTService.PRIMARY_SERVICE_UUID)
            for service in connection.get_services():
                if service.uuid == YASP_SRV_UUID:
                    connection.find_information(service=service)
                    connection.read_by_type(service=service, type=GATTCharacteristic.CHARACTERISTIC_UUID)
                    connection.read_by_type(service=service, type=GATTCharacteristic.CLIENT_CHARACTERISTIC_CONFIG)
                    break
            else:
                raise Exception("Could not find YASP_SRV_UUID in device services!")
            logger.log(logging.INFO, "YASP service discovery complete.")
            for characteristic in connection.get_characteristics():
                if characteristic.has_notify():
                    connection.characteristic_subscription(characteristic=characteristic, indicate=False, notify=True)
            logger.log(logging.INFO, "YASP characteristic notifications enabled.")
            serial_chr_handle = connection.get_handles_by_uuid(YASP_CHR_UUID)[0]
            yasp_client.set_interface(serial_chr_handle, connection.wr_noresp_by_handle)
            connection.assign_attrclient_value_callback(serial_chr_handle, yasp_client.serial_rx)
            return connection.handle
        else:
            return None

    def ble_evt_gap_scan_response(self, rssi, packet_type, sender, address_type, bond, data):
        super(YaspBlueGigaClient, self).ble_evt_gap_scan_response(rssi, packet_type, sender, address_type, bond, data)
        result = BLEScanResponse(rssi, packet_type, sender, address_type, bond, data)
        if self.scan_filter_name is not None and self.scan_filter_service is not None:
            if self.scan_filter_name in data and self.scan_filter_service in data:
                self.scan_filter_result = result
        elif self.scan_filter_name is not None:
            if self.scan_filter_name in data:
                self.scan_filter_result = result
        elif self.scan_filter_service is not None:
            if self.scan_filter_service in data:
                self.scan_filter_result = result


class YaspBlueGigaServer(BlueGigaServer):
    def __init__(self, port):
        super(YaspBlueGigaServer, self).__init__(port)
        self.pipe_logs_to_terminal(level=logging.WARNING)
        self.yasp_server = None
        self.connected = False
        self.yasp_chr_handle = None
        self.keep_running = True
        for handle in range(0x00FF):
            handle_type = self.read_type(handle)
            if handle_type == YASP_CHR_UUID:
                print("Found YASP_CHR_UUID @ %d." % handle)
                self.yasp_chr_handle = handle
                break
        else:
            raise RuntimeError("Could not find required YASP characteristic.")

    def start_yasp_server(self, yasp_server):
        self.yasp_server = yasp_server
        self.yasp_server.set_interface(handle=self.yasp_chr_handle, wr_wo_resp=self.write_attribute)
        self.keep_running = True
        self.advertise_general()

    def stop_yasp_server(self):
        self.keep_running = False
        self.reset_ble_state()

    def ble_evt_attributes_value(self, connection, reason, handle, offset, value):
        super(YaspBlueGigaServer, self).ble_evt_attributes_value(connection, reason, handle, offset, value)
        if handle == self.yasp_chr_handle:
            self.yasp_server.serial_rx(value)

    def ble_evt_connection_disconnected(self, connection, reason):
        super(YaspBlueGigaServer, self).ble_evt_connection_disconnected(connection, reason)
        self.connected = False
        if self.keep_running:
            self.start_yasp_server(self.yasp_server)

    def ble_evt_connection_status(self, connection, flags, address, address_type, conn_interval, timeout, latency, bonding):
        super(YaspBlueGigaServer, self).ble_evt_connection_status(connection, flags, address, address_type, conn_interval, timeout, latency, bonding)
        if flags & connection_status_mask["connection_connected"]:
            self.connected = True


def main():
    pass


if __name__ == "__main__":
    main()
