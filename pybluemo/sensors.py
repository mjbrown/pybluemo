import time
from .message import MsgAccelStream, EnumAccelDataRate


class Bma400StreamHandler(object):
    def __init__(self, yasp_client=None, csv_filename=None):
        if csv_filename is not None:
            self.csv_fp = open(csv_filename, "w")
        else:
            self.csv_fp = None
        if yasp_client is not None:
            self.yasp_client = yasp_client
            yasp_client.set_default_msg_callback(MsgAccelStream.get_response_code(), self.accel_data_callback)
        else:
            self.yasp_client = None
        self.last_t = time.time()

    @classmethod
    def accel_to_float(cls, range, short, bits):
        if short & (1 << (bits - 1)):
            value = short - (1 << (bits))
        else:
            value = short
        range_scaler = float(2 << range) * 9.8 / (1 << (bits - 1))
        return float(value) * range_scaler

    def parsed_data_ready(self, x, y, z, t):
        if self.csv_fp is not None:
            self.csv_fp.write("%s,%f,%f,%f\n" % (t, x, y, z))
        print("Parsed X:%f Y:%f Z:%f" % (x, y, z))

    def accel_data_callback(self, msg):
        range = msg.get_param("DataRange")
        rate = msg.get_param("DataRate")
        if rate == EnumAccelDataRate.F800:
            delta = 1/800
        elif rate == EnumAccelDataRate.F400:
            delta = 1/400
        elif rate == EnumAccelDataRate.F200:
            delta = 1/200
        elif rate == EnumAccelDataRate.F100:
            delta = 1/100
        elif rate == EnumAccelDataRate.F50:
            delta = 1/50
        elif rate == EnumAccelDataRate.F25:
            delta = 1/25
        else:
            delta = 1/12.5
        data = msg.get_param("AccelData")
        print("Range:%d Rate:%d Data:%s" % (range, rate, "".join(["%02X" % i for i in data])))
        if abs(self.last_t - time.time()) > 1:
            self.last_t = time.time()
        while len(data) > 0:
            if data[0] & 0xE0 == 0x80:  # Sensor data frame
                if data[0] & 0x0F != 0x0E:
                    raise Exception("You must enable all three accelerometer axes.")
                if data[0] & 0x10 == 1 and len(data) >= 4:
                    self.parsed_data_ready(
                        self.accel_to_float(range, data[1], 8),
                        self.accel_to_float(range, data[2], 8),
                        self.accel_to_float(range, data[3], 8),
                        self.last_t)
                    data = data[4:]
                elif len(data) >= 7:
                    self.parsed_data_ready(
                        self.accel_to_float(range, (data[2] << 4) + data[1], 12),
                        self.accel_to_float(range, (data[4] << 4) + data[3], 12),
                        self.accel_to_float(range, (data[6] << 4) + data[5], 12),
                        self.last_t)
                    data = data[7:]
                else:
                    data = b""
                self.last_t += delta
            elif data[0] & 0xE0 == 0xA0:  # Sensor time frame
                if len(data) >= 4:
                    data = data[4:]
                else:
                    data = b""
            elif data[0] == 0x48:  # Control frame
                if len(data) >= 2:
                    data = data[2:]
                else:
                    data = b""
