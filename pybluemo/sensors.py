import time
import datetime
from .message import MsgAccelStream, EnumAccelDataRate
from .sensordb import CreateDataStreamsInput


class Bma400StreamHandler(object):
    def __init__(self, yasp_client=None, csv_filename=None, cloud_logger=None, upload_period=10):
        if csv_filename is not None:
            self.csv_fp = open(csv_filename, "w")
        else:
            self.csv_fp = None
        if yasp_client is not None:
            self.yasp_client = yasp_client
            yasp_client.set_default_msg_callback(MsgAccelStream.get_response_code(), self.accel_data_callback)
        else:
            self.yasp_client = None
        self.first_t = None
        self.samples = 0
        self.x_hist = []
        self.y_hist = []
        self.z_hist = []
        self.cloud_logger = cloud_logger
        self.upload_period = upload_period
        self.last_upload = time.time()
        self.prev_end = None

    @classmethod
    def accel_to_float(cls, range, short, bits):
        if short & (1 << (bits - 1)):
            value = short - (1 << bits)
        else:
            value = short
        range_scaler = float(2 << range) * 9.8 / (1 << (bits - 1))
        return float(value) * range_scaler

    def parsed_data_ready(self, x, y, z, period):
        if self.csv_fp is not None:
            self.csv_fp.write("%f,%f,%f\n" % (x, y, z))
        if self.cloud_logger is not None:
            self.x_hist.append(x)
            self.y_hist.append(y)
            self.z_hist.append(z)
            if self.last_upload + self.upload_period < time.time():
                if (not self.prev_end) or (time.time() - (self.prev_end + period * len(self.x_hist))) > 1:
                    print("Setting self.prev_end...")
                    self.prev_end = time.time() - period * len(self.x_hist)
                this_end = self.prev_end + period * len(self.x_hist)
                for tup in [("X", self.x_hist), ("Y", self.y_hist), ("Z", self.z_hist)]:
                    self.cloud_logger.log_stream(start=datetime.datetime.utcfromtimestamp(self.prev_end),
                                                 end=datetime.datetime.utcfromtimestamp(this_end),
                                                 label="Accel", channel=tup[0], frequency=1/period, unit="m/s^2",
                                                 values=tup[1])
                self.prev_end = this_end
                self.x_hist = []
                self.y_hist = []
                self.z_hist = []
                self.last_upload = time.time()
        #print("Parsed X:%f Y:%f Z:%f" % (x, y, z))

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
        if self.csv_fp is not None:
            if self.first_t is None:
                self.csv_fp.write(datetime.datetime.utcnow().isoformat() + "Z\n")
                self.csv_fp.write("Range:%d Period:%f\n" % (range, delta))
                self.first_t = time.time()
        #print("Range:%d Rate:%d Data:%s" % (range, rate, "".join(["%02X" % i for i in data])))
        while len(data) > 0:
            if data[0] & 0xE0 == 0x80:  # Sensor data frame
                if data[0] & 0x0F != 0x0E:
                    raise Exception("You must enable all three accelerometer axes.")
                if (data[0] & 0x10) == 0x10 and len(data) >= 4:
                    self.parsed_data_ready(
                        self.accel_to_float(range, data[1], 8),
                        self.accel_to_float(range, data[2], 8),
                        self.accel_to_float(range, data[3], 8),
                        delta)
                    data = data[4:]
                elif len(data) >= 7:
                    self.parsed_data_ready(
                        self.accel_to_float(range, (data[2] << 4) + data[1], 12),
                        self.accel_to_float(range, (data[4] << 4) + data[3], 12),
                        self.accel_to_float(range, (data[6] << 4) + data[5], 12),
                        delta)
                    data = data[7:]
                else:
                    data = b""
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
