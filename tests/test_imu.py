from pybluemo.bluemo_ble import *
from pybluemo.bluemo_msg import *
import math
import struct

GRAVITY = 9.80655


def to_radians(degrees):
    return (float(degrees) / 360.0) * math.pi * 2.0


class Vector(object):
    def __init__(self, x=0.0, y=0.0, z=0.0):
        self.x = x
        self.y = y
        self.z = z

    def magnitude(self):
        return math.sqrt(self.x * self.x + self.y * self.y + self.z * self.z)

    def normalize(self):
        mag = self.magnitude()
        if mag > 0:
            return self * (1 / mag)
        else:
            return self * 1.0

    def dot(self, vector):
        return vector.x * self.x + vector.y * self.y + vector.z * self.z

    def cross(self, vector):
        v = Vector()
        v.x = self.y * vector.z - self.z * vector.y
        v.y = self.z * vector.x - self.x * vector.z
        v.z = self.x * vector.y - self.y * vector.x
        return v

    def angle(self, vector):
        return math.acos(self.dot(vector) / (self.magnitude() * vector.magnitude()))

    def __add__(self, other):
        v = Vector()
        v.x = self.x + other.x
        v.y = self.y + other.y
        v.z = self.z + other.z
        return v

    def __sub__(self, other):
        v = Vector()
        v.x = self.x - other.x
        v.y = self.y - other.y
        v.z = self.z - other.z
        return v

    def __mul__(self, other):
        v = Vector()
        if isinstance(other, self.__class__):
            v.x = self.x * other.x
            v.y = self.y * other.y
            v.z = self.z * other.z
        else:
            v.x = self.x * other
            v.y = self.y * other
            v.z = self.z * other
        return v

    def __str__(self):
        return "x:%f, y:%f, z:%f" % (self.x, self.y, self.z)


class Quaternion(object):
    def __init__(self, x=1.0, y=0.0, z=0.0, radians=0.0):
        self.w = math.cos(radians / 2)
        self.x = float(x) * math.sin(radians / 2)
        self.y = float(y) * math.sin(radians / 2)
        self.z = float(z) * math.sin(radians / 2)

    def __str__(self):
        return "w:%f, x:%f, y:%f, z:%f" % (self.w, self.x, self.y, self.z)

    def normalize(self):
        mag2 = self.w * self.w + self.x * self.x + self.y * self.y + self.z * self.z
        if abs(mag2 - 1.0) > 0.00001:
            mag = math.sqrt(mag2)
            self.w /= mag
            self.x /= mag
            self.y /= mag
            self.z /= mag

    def inverted(self):
        q = Quaternion()
        q.w = self.w
        q.x = -self.x
        q.y = -self.y
        q.z = -self.z
        return q

    @classmethod
    def q_multiply(cls, q1, q2):
        q = Quaternion()
        q.w = q1.w * q2.w - q1.x * q2.x - q1.y * q2.y - q1.z * q2.z
        q.x = q1.w * q2.x + q1.x * q2.w + q1.y * q2.z - q1.z * q2.y
        q.y = q1.w * q2.y + q1.y * q2.w + q1.z * q2.x - q1.x * q2.z
        q.z = q1.w * q2.z + q1.z * q2.w + q1.x * q2.y - q1.y * q2.x
        return q

    @classmethod
    def qv_mult(cls, q1, vector):
        q2 = Quaternion()
        q2.w = 0
        q2.x = vector.x
        q2.y = vector.y
        q2.z = vector.z
        result = cls.q_multiply(q1, cls.q_multiply(q2, q1.inverted()))
        return Vector(result.x, result.y, result.z)


class Position(object):
    def __init__(self):
        self.calibration = Vector(0.0, 0.0, 0.0)
        self.position = Vector(0.0, 0.0, 0.0)
        self.velocity = Vector(0.0, 0.0, 0.0)
        self.rotation = Quaternion(1.0, 0.0, 0.0, to_radians(1.0))

    def cb_orientation(self, msg_defn):
        q = Quaternion()
        q.w = msg_defn.get_param("Qw")
        q.x = msg_defn.get_param("Qx")
        q.y = msg_defn.get_param("Qy")
        q.z = msg_defn.get_param("Qz")
        #print("FW Quaternion: %s" % q)
        if q.w <= 1.0:
            print("FW Orient: %f - %s" % (2 * math.acos(q.w), Vector(q.x, q.y, q.z).normalize()))

    # 2G: -.01, -.2087, -.309
    # 4G: -.13, -.205, -.31
    def cb_accel_stream(self, msg_defn):
        data = msg_defn.get_param("AccelData")
        samples = struct.unpack("<%dh" % (len(data) / 2), data)
        data_range = msg_defn.get_param("DataRateAndRange") >> 4
        data_rate = msg_defn.get_param("DataRateAndRange") & 0xF
        range_unit = (2 << data_range) * 9.8 / (1 << 13)
        rate_unit = 1.0 / (125.0 * math.pow(2, data_rate - 4))
        for sample in range(len(samples) / 3):
            x_acc = float(samples[3*sample]) * range_unit
            y_acc = float(samples[3*sample+1]) * range_unit
            z_acc = float(samples[3*sample+2]) * range_unit
            accel = Vector(x_acc, y_acc, z_acc) + self.calibration
            if accel.magnitude() > (GRAVITY + 0.2):
                self.calibration -= accel * 0.0001
            elif accel.magnitude() < -(GRAVITY + 0.2):
                self.calibration += accel * 0.0001
            else:
                orientation = Vector(self.rotation.x, self.rotation.y, self.rotation.z)
                angle = accel.angle(orientation)
                cross = accel.cross(orientation)
                adjustment = Quaternion(cross.x, cross.y, cross.z, angle / 10)
                #print("Adjustment: %s" % adjustment)
                self.rotation = Quaternion.q_multiply(self.rotation, adjustment)
                self.rotation.normalize()

            #print("Acceleration(%f): %s" % (accel.magnitude(), accel))
            v_accel = Quaternion.qv_mult(self.rotation, accel) - Vector(0, 0, -9.80665)
            self.position += (self.velocity * rate_unit) + v_accel * rate_unit * rate_unit * 2.0
            self.velocity += v_accel * rate_unit
        #print("Calibration: %s" % self.calibration)
        #print("Rotation: %s" % self.rotation)

    def cb_gyro_stream(self, msg_defn):
        data = msg_defn.get_param("GyroData")
        data_range = msg_defn.get_param("DataRateAndRange") >> 4
        data_rate = msg_defn.get_param("DataRateAndRange") & 0x0F
        samples = struct.unpack("<%dh" % (len(data) / 2), data)
        rate_unit = 1.0 / (100.0 * math.pow(2.0, (data_rate - 3)))
        range_unit = (2000.0 / float(math.pow(2, data_range))) / float(1 << 15)
        data_points = len(samples) / 3
        for sample in range(data_points):
            roll = float(samples[3*sample]) * range_unit + 0.08
            q_roll = Quaternion(1, 0, 0, to_radians(roll * rate_unit))

            pitch = float(samples[3*sample+1]) * range_unit + 0.804
            q_pitch = Quaternion(0, 1, 0, to_radians(pitch * rate_unit))

            yaw = float(samples[3*sample+2]) * range_unit + 0.273
            q_yaw = Quaternion(0, 0, 1, to_radians(yaw * rate_unit))

            self.rotation = Quaternion.q_multiply(Quaternion.q_multiply(Quaternion.q_multiply(self.rotation, q_roll), q_yaw), q_pitch)
        self.rotation.normalize()
        orientation = Vector(self.rotation.x, self.rotation.y, self.rotation.z).normalize()
        print("Orientation: %f - %s" % (2 * math.acos(self.rotation.w), orientation))
        #print("Rotation: %s" % self.rotation)


def test_gyro_stream(yasp_client):
    msg = MsgGyroStream().set_param("DataRange", 0x2).set_param("DataRate", 0x01).set_param("Watermark", 4)
    resp = yasp_client.send_command(callback=None, msg_defn=msg)
    time.sleep(30)
    msg.set_param("DataRate", 0)
    resp = yasp_client.send_command(callback=None, msg_defn=msg)


def test_accel_stream(yasp_client):
    msg = MsgAccelStream().set_param("DataRange", 0x01).set_param("DataRate", 0x03).set_param("Watermark", 4)
    resp = yasp_client.send_command(callback=None, msg_defn=msg)
    time.sleep(30)
    msg.set_param("DataRate", 0)
    resp = yasp_client.send_command(callback=None, msg_defn=msg)


def test_full_imu(yasp_client):
    accel_msg = MsgAccelStream().set_param("DataRange", 0x1).set_param("DataRate", 0x01).set_param("Watermark", 2)
    resp = yasp_client.send_command(callback=None, msg_defn=accel_msg)
    gyro_msg = MsgGyroStream().set_param("DataRange", 0x2).set_param("DataRate", 0x01).set_param("Watermark", 4)
    resp = yasp_client.send_command(callback=None, msg_defn=gyro_msg)
    time.sleep(60)


def test_orientation(yasp_client):
    msg = MsgStartOrientation().set_param("GyroRate", 0x01).set_param("GyroRange", 0x02).set_param("GyroWatermark", 4)
    resp = yasp_client.send_command(callback=None, msg_defn=msg)
    time.sleep(60)
    msg.set_param("GyroRate", 0x00)
    resp = yasp_client.send_command(callback=None, msg_defn=msg)


def test_fixed(yasp_client):
    msg = MsgLedConfig().set_param("Instance", 0) \
        .set_param("Modify", 1) \
        .set_param("Module", 5) \
        .set_param("PinIndex", 0) \
        .set_param("LedLength", 60) \
        .set_param("AnimationRate", 10) \
        .set_param("AnimationType", MsgLedConfig.ANIM_ROT_INDIVIDUAL)
    resp = yasp_client.send_command(callback=None, msg_defn=msg)
    msg = MsgLedPattern().set_param("Instance", 0) \
        .set_param("Modify", 1) \
        .add_run_length(0, 30, "\x00\x00\x00") \
        .add_run_length(0, 15, "\x10\x10\x00", rle_type=MsgLedPattern.TYPE_DESCENDING, horiz=1) \
        .add_run_length(15, 15, "\x00\x00\x10", rle_type=MsgLedPattern.TYPE_REV_DESC, horiz=1)
    resp = yasp_client.send_command(callback=None, msg_defn=msg)


def main():
    client = YaspBlueGigaClient(port="COM9")
    client.reset_ble_state()
    #client.pipe_logs_to_terminal(level=logging.INFO)
    yasp_client = YaspClient()
    position = Position()
    conn_handle = client.connect_by_name("Unit 01", yasp_client)
    yasp_client.set_default_msg_callback(MsgGyroStream.get_response_code(), position.cb_gyro_stream)
    yasp_client.set_default_msg_callback(MsgAccelStream.get_response_code(), position.cb_accel_stream)
    yasp_client.set_default_msg_callback(MsgStartOrientation.get_response_code(), position.cb_orientation)
    #test_gyro_stream(yasp_client)
    #test_accel_stream(yasp_client)
    #test_full_imu(yasp_client)
    test_fixed(yasp_client)
    time.sleep(0.1)
    client.disconnect(conn_handle)


def quaternion_testing():
    orientation = Vector(1, 0, 0)
    q1 = Quaternion(0, 1, 0, to_radians(90))
    q2 = Quaternion(1, 0, 0, to_radians(45))
    q3 = Quaternion.q_multiply(q2, q1)
    r1 = Quaternion.qv_mult(q1, orientation)
    print("Quaternion 1: %s - Result: %s" % (q1, r1))
    r2 = Quaternion.qv_mult(q2, orientation)
    print("Quaternion 2: %s - Result: %s" % (q2, r2))
    r3 = Quaternion.qv_mult(q3, orientation)
    print("Quaternion 3: %s - Result: %s" % (q3, r3))
    r4 = Quaternion.qv_mult(q2, r1)
    print("Result 4: %s" % r4)


if __name__ == "__main__":
    main()
