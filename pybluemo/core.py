import logging
import sys
from abc import ABCMeta, abstractmethod
import struct
import time
import threading

from bgapi.module import BlueGigaClient, GATTService, GATTCharacteristic
from bgapi.module import ProcedureManager, BLEScanResponse
from bgapi.cmd_def import gap_discover_mode

from .message import MessageDefinition

term = logging.StreamHandler(sys.stdout)
formatter = logging.Formatter('%(asctime)s - %(name)s - %(levelname)s - %(message)s')
term.setFormatter(formatter)
logger = logging.getLogger("YASP")
logger.addHandler(term)
logger.setLevel(logging.INFO)


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
        self.buffer_lock = threading.Semaphore(1)

    @abstractmethod
    def serial_tx(self, data): raise NotImplementedError()

    def serial_rx(self, data):
        logger.log(logging.DEBUG, "<=" + "".join(["%02X" % j for j in data]))
        self.buffer_lock.acquire(True)
        self.buffer += data
        self.buffer_lock.release()
        self.rx_processing()

    def rx_processing(self):
        if len(self.buffer) > 1:
            hdr_len = 2
            length = self.buffer[0]
            if length > 127:
                if len(self.buffer) > hdr_len:
                    length = (self.buffer[0] & 0x7F) + ((self.buffer[1] & 0xF) << 7)
                    if self.buffer[1] > 0xF:
                        logger.error("RX Overrun, packets dropped.")
                    hdr_len += 1
                else:
                    return
            if len(self.buffer) >= hdr_len + length:
                cmd = self.buffer[hdr_len-1]
                message = self.buffer[hdr_len:hdr_len+length]
                threading.Thread(target=self.cmd_handler, args=(cmd, message)).start()
                self.buffer_lock.acquire(True)
                self.buffer = self.buffer[hdr_len+length:]
                self.buffer_lock.release()
                self.rx_processing()

    def send_command(self, callback, msg_defn=MessageDefinition(), timeout=1):
        payload = msg_defn.cmd_msg()
        if len(payload) > 0x7F:
            msg = struct.pack("<BBB%ds" % len(payload), (len(payload) & 0x7F) | 0x80, len(payload) >> 7, msg_defn.get_command_code(), payload)
        elif len(payload) > 0:
            msg = struct.pack("<BB%ds" % len(payload), len(payload), msg_defn.get_command_code(), payload)
        else:
            msg = struct.pack("<BB", len(payload), msg_defn.get_command_code())
        if callback is not None:
            if msg_defn.get_response_code() not in self.cmd_callbacks:
                self.cmd_callbacks[msg_defn.get_response_code()] = []
            self.cmd_callbacks[msg_defn.get_response_code()].append(callback)
            self.serial_tx(msg)
        elif msg_defn.get_response_code() in self.def_callbacks:
            self.serial_tx(msg)
        else:
            with self.procedure_call(msg_defn.get_response_code(), timeout=timeout):
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
    def __init__(self, msg_class_lookup):
        super(YaspClient, self).__init__()
        self.msg_class_lookup = msg_class_lookup;
        for key, value in msg_class_lookup.items():
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
            logger.log(logging.DEBUG, "=>"+"".join(["%02X" % j for j in data[YASP_BLE_MTU*i:YASP_BLE_MTU*(i+1)]]))
            self.write_wo_response(self.serial_char_handle, data[YASP_BLE_MTU*i:YASP_BLE_MTU*(i+1)], attempts=3)
        if (len(data) % YASP_BLE_MTU) > 0:
            logger.log(logging.DEBUG, "=>" + "".join(["%02X" % j for j in data[YASP_BLE_MTU*full_packets:]]))
            self.write_wo_response(self.serial_char_handle, data[YASP_BLE_MTU*full_packets:], attempts=3)

    def cmd_handler(self, cmd, payload):
        if cmd not in self.msg_class_lookup:
            raise RuntimeError()
        response = self.msg_class_lookup[cmd].rsp_recv(payload)
        if len(self.cmd_callbacks[cmd]) == 0:       # Callback not specified
            if cmd in self.def_callbacks:           # Default callback specified
                self.def_callbacks[cmd](response)
            else:
                self.cached_response = response
                self.procedure_complete(cmd)
        else:
            callback = self.cmd_callbacks[cmd].pop()
            callback(response)


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
            logger.log(logging.INFO, "Intiating connection to %s" % ":".join(["%02X" % i for i in self.scan_filter_result.sender[::-1]]))
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
        print("Advertisement data: " + "".join(["%c" % i for i in data]))
        if self.scan_filter_name is not None and self.scan_filter_service is not None:
            if self.scan_filter_name in data and self.scan_filter_service in data:
                self.scan_filter_result = result
        elif self.scan_filter_name is not None:
            if self.scan_filter_name in data:
                self.scan_filter_result = result
        elif self.scan_filter_service is not None:
            if self.scan_filter_service in data:
                self.scan_filter_result = result

