import bgapi.module
from pybluemo.bluemo_msg import *
from pybluemo.bluemo_ble import *
import math
import time
from asyncio import Semaphore

loop_semaphore = Semaphore(2)

DEVICE_NAME = b"Bluemo01"


def cb_error(msg_defn):
    print("Remote Error: ", "".join(["%c" % i for i in msg_defn.get_param("Message")]))


def config_cb(msg):
    print("Device Config: - %s - BatteryType:%d - HardwareRevision:%d - SerialNumber: %d" %
          (msg.get_param("DeviceName"), msg.get_param("BatteryType"),
           msg.get_param("HardwareRevision"), msg.get_param("SerialNumber")))


def test_device_config(yasp_client):
    msg = MsgDeviceConfig().set_param("DeviceName", "Bluemo03")\
                           .set_param("SerialNumber", 3)\
                           .set_param("HardwareRevision", 1)\
                           .set_param("BatteryType", 1)\
                           .set_param("Modify", 1)
    yasp_client.send_command(callback=config_cb, msg_defn=msg)
    time.sleep(1)


def test_soft_reset(client, yasp_client):
    msg = MsgSoftReset()
    try:
        yasp_client.send_command(callback=None, msg_defn=msg)
    except bgapi.module.Timeout:
        pass
    time.sleep(5)
    client.connect_by_name(DEVICE_NAME, yasp_client, scan_timeout=2)


def loopback_cb(msg_defn):
    print("Callback Stub.", msg_defn)
    loop_semaphore.release()


def test_loopback(yasp_client):
    TEST_ITERATIONS = 1000
    TEST_VECTOR = b"\xDE\xAD\xBE\xEF\xBA\xAD\xF0\x0D" * 31
    time.sleep(1)
    start = time.time()
    for i in range(TEST_ITERATIONS):
        loop_semaphore.acquire(blocking=True)
        msg = MsgLoopback().set_param("Handle", 0).set_param("Payload", TEST_VECTOR[0:i % len(TEST_VECTOR)])
        yasp_client.send_command(callback=loopback_cb, msg_defn=msg)
    end = time.time()
    time.sleep(1)
    print("Total: %d - Speed: %d bytes/sec" % (len(TEST_VECTOR) * TEST_ITERATIONS,
                                               len(TEST_VECTOR) * TEST_ITERATIONS / (end - start)))


def detect_cb(msg):
    print("Detect callback - %d - %04X" % (msg.get_param("Module"), msg.get_param("DetectResult")))


def test_detect_module(yasp_client):
    msg = MsgDetectModule()
    for i in range(4):
        resp = yasp_client.send_command(callback=detect_cb, msg_defn=msg.set_param("Module", i))
        time.sleep(1)
    time.sleep(3)


def test_flash_init(yasp_client):
    msg = MsgFlashInitialize().set_param("ChipSelectPin", 16).set_param("MosiPin", 4)\
                              .set_param("MisoPin", 15).set_param("SckPin", 17)
    resp = yasp_client.send_command(callback=None, msg_defn=msg)


def verify_flash(addr, data, yasp_client):
    CHUNK_SIZE = 256
    chunks = len(data) // CHUNK_SIZE
    if len(data) % CHUNK_SIZE > 0:
        chunks += 1
    for i in range(chunks):
        msg = MsgFlashRead().set_param("Address", addr + (i*CHUNK_SIZE))\
            .set_param("Length", len(data[i*CHUNK_SIZE:(i+1)*CHUNK_SIZE]))
        resp = yasp_client.send_command(callback=None, msg_defn=msg)
        for j in range(len(resp.get_param("Data"))):
            if resp.get_param("Data")[j] != data[i*CHUNK_SIZE + j]:
                logger.log(logging.ERROR, "Verify Error: %s")
                return


def write_flash(addr, data, yasp_client):
    CHUNK_SIZE = 256
    chunks = len(data) // CHUNK_SIZE
    if len(data) % CHUNK_SIZE > 0:
        chunks += 1
    for i in range(chunks):
        msg = MsgFlashWrite().set_param("Address", addr + (i*CHUNK_SIZE)).set_param("Data", data[i*CHUNK_SIZE:(i+1)*CHUNK_SIZE])
        resp = yasp_client.send_command(callback=None, msg_defn=msg)


def test_flash_operations(yasp_client):
    with open("testdata.spiflash", "rb") as spidata:
        data = b""
        for line in spidata.readlines():
            data += b"%c" % int(line, 16)
        print("Data Length: %d" % len(data))

        # Erase
        msg = MsgFlashErase().set_param("Address", 0x1000).set_param("Length", len(data))
        resp = yasp_client.send_command(callback=None, msg_defn=msg)

        # Verify Erase
        verify_flash(0x1000, len(data)*b"\xFF", yasp_client)

        # Write
        write_flash(0x1000, data, yasp_client)

        # Verify Write
        verify_flash(0x1000, data, yasp_client)


def test_flash_uninit(yasp_client):
    msg = MsgFlashDisconnect()
    resp = yasp_client.send_command(callback=None, msg_defn=msg)


def twim_scan_cb(msg):
    logger.log(logging.INFO, "TWIM Scan Result - Slave Address: %02X - Total Found: %d" %
               (msg.get_param("SlaveAddress"), msg.get_param("TotalFound")))


def test_twim_scan(yasp_client):
    yasp_client.set_default_msg_callback(MsgTwimScan.get_response_code(), twim_scan_cb)
    msg = MsgTwimScan().set_param("TestByte", 0)
    resp = yasp_client.send_command(callback=None, msg_defn=msg)
    time.sleep(3)


def pin_function_cb(msg):
    logger.log(logging.INFO, "Pin Function - Module:%d - Pin:%d - Function:%d - Instance:%d" %
                             (msg.get_param("Module"), msg.get_param("PinIndex"),
                              msg.get_param("Function"), msg.get_param("Instance")))


def test_pin_function(yasp_client):
    msg = MsgPinFunction().set_param("Module", 5).set_param("PinIndex", 0)\
        .set_param("Function", MsgPinFunction.FUNC_WS2812)\
        .set_param("Instance", 0).set_param("SaveConfig", 1)
    resp = yasp_client.send_command(callback=None, msg_defn=msg)
    time.sleep(1)
    #msg = MsgResourceConfiguration().set_param("RestorePinConfig", 1)
    #resp = yasp_client.send_command(callback=None, msg_defn=msg)
    #time.sleep(1)


def test_ws2812(yasp_client):
    strip_length = 30
    msg = MsgLedConfig().set_param("Instance", 0).set_param("SaveConfig", 1).set_param("LedLength", strip_length)\
        .set_param("AnimationRate", 1).set_param("AnimationType", 1)\
        .add_run_length(0, 6, "\x00\x0F\x07", rle_type=MsgLedPattern.TYPE_ASCENDING)
    yasp_client.send_command(callback=None, msg_defn=msg)
    msg = MsgLedPattern().set_param("Instance", 0).set_param("Offset", 0)
#    msg.add_run_length(0, strip_length, "\x00\x00\xFF", rle_type=MsgLedPattern.TYPE_REV_DESC)
    msg.add_run_length(strip_length/5, 6, "\x00\x0F\x07", rle_type=MsgLedPattern.TYPE_ASCENDING) \
        .add_run_length((2*strip_length/5) % strip_length, 6, "\x0F\x07\x20", rle_type=MsgLedPattern.TYPE_ASCENDING) \
       .add_run_length((3*strip_length/5) % strip_length, 6, "\x0F\x1F\x00", rle_type=MsgLedPattern.TYPE_ASCENDING) \
       .add_run_length((4*strip_length/5) % strip_length, 6, "\x1F\x1F\x1F", rle_type=MsgLedPattern.TYPE_ASCENDING) \
       .add_run_length((5*strip_length/5) % strip_length, 6, "\x1F\x00\x0F", rle_type=MsgLedPattern.TYPE_ASCENDING)
    yasp_client.send_command(callback=None, msg_defn=msg)


def servo_config_cb(msg):
    print("Servo Config %d-%d" % (msg.get_param("Module"), msg.get_param("Channel")))
    for key in msg.parameters.iterkeys():
        try:
            print("%s:%d" % (key, msg.get_param(key)))
        except:
            print("ServoName: %s" % msg.get_param("ServoName"))


def test_servo_control(yasp_client):
    for channel in range(8):
        msg = MsgServoConfig().set_param("Module", 0).set_param("Channel", channel).set_param("ServoName", "Yolo %d" % channel)
        resp = yasp_client.send_command(callback=None, msg_defn=msg)
        #print("Servo name: %s" % resp.get_param("ServoName"))
        time.sleep(.1)
    msg = MsgServoControl().set_param("Module", 0).set_param("Channel", 0).set_param("PulseWidth", 1500)
    resp = yasp_client.send_command(callback=None, msg_defn=msg)
    time.sleep(1)
    msg = MsgServoConfig().set_param("Module", 0).set_param("Channel", 0xFE).set_param("ServoName", "")
    resp = yasp_client.send_command(callback=None, msg_defn=msg)
    time.sleep(1)


def test_robotis(yasp_client):
    msg = MsgRobotisConfig().set_param("Module", 0).set_param("ServoId", 0x30).set_param("ServoName", "Yolo")
    resp = yasp_client.send_command(callback=None, msg_defn=msg)
    print("Servo Name: %s" % resp.get_param("ServoName"))
    for cmd in ["\x1E\xAF\x00"]:
        for id in [0x30, 0x31, 0x32]:
            msg = MsgRobotisCommand().set_param("Module", 0).set_param("ServoId", id).set_param("Instruction", 3).set_param("Parameters", cmd)
            resp = yasp_client.send_command(callback=None, msg_defn=msg)
        time.sleep(1)


def cb_battery_control(resp):
    print("Battery Control - OTG:%d - CHG:%d - Vbatt:%f - Vsys:%f - Vbus:%f - Ichg:%f" %
          (resp.get_param("OtgEnableStatus"), resp.get_param("ChargingEnableStatus"),
          resp.get_battery_voltage(), resp.get_system_voltage(), resp.get_usb_voltage(), resp.get_charge_current()))


def test_battery_control(yasp_client):
    for i in range(10):
        msg = MsgBatteryStatus().set_param("OtgEnable", 0).set_param("OtgDisable", 0).set_param("ChargingEnable", 0).set_param("ChargingDisable", 0)
        resp = yasp_client.send_command(callback=None, msg_defn=msg)
        time.sleep(1)


def cb_stream_accelerometer(msg_defn):
    rate_and_range = msg_defn.get_param("DataRateAndRange")
    data_rate = rate_and_range & 0xF
    data_range = (4 << (rate_and_range >> 4)) * 1.0
    data = msg_defn.get_param("AccelData")
    samples = struct.unpack("<%dh" % (len(data)/2), data)
    parsed = []
    for i in range(len(samples) / 3):
        parsed.append((samples[3*i], samples[3*i+1], samples[3*i+2],
                     (data_range / (1 << 14)) * math.sqrt(samples[3*i]*samples[3*i] + samples[3*i+1]*samples[3*i+1] + samples[3*i+2]*samples[3*i+2])))
    print("Accelerometer Data Received", parsed)


def test_stream_accelerometer(yasp_client):
    msg = MsgAccelStream().set_param("DataRange", 0x00).set_param("DataRate", 0x01).set_param("Watermark", 1)
    resp = yasp_client.send_command(callback=None, msg_defn=msg)
    time.sleep(10)
    msg.set_param("DataRate", 0)
    resp = yasp_client.send_command(callback=None, msg_defn=msg)


def cb_gyro_stream(msg_defn):
    data = msg_defn.get_param("GyroData")
    samples = struct.unpack("<%dh" % (len(data)/2), data)
    print("Gyro Data Received", samples)


def test_gyro_stream(yasp_client):
    msg = MsgGyroStream().set_param("DataRange", 0x4).set_param("DataRate", 0x08).set_param("Watermark", 16)
    resp = yasp_client.send_command(callback=None, msg_defn=msg)
    time.sleep(30)
    msg.set_param("DataRate", 0)
    resp = yasp_client.send_command(callback=None, msg_defn=msg)


def test_motor_control(yasp_client):
    msg = MsgMotorControl().set_param("Module", 0).set_param("Channel", 0)
    for duty_cycle in range(30):
        resp = yasp_client.send_command(callback=None, msg_defn=msg.set_param("Channel", 0).set_param("DutyCycle", duty_cycle*1000 | 0x8000))
        resp = yasp_client.send_command(callback=None, msg_defn=msg.set_param("Channel", 1))
        resp = yasp_client.send_command(callback=None, msg_defn=msg.set_param("Channel", 2))
        resp = yasp_client.send_command(callback=None, msg_defn=msg.set_param("Channel", 3))
    time.sleep(1)


def test_gpio_expander_control(yasp_client, handle):
    msg = MsgGpioExpanderConfig().set_param("Module", handle).set_param("Channel", 0).set_param("InputEnabled", 0).set_param("OutputEnabled", 0).set_param("OutputDefault", 0).set_param("ChannelName", "Yolo")
    resp = yasp_client.send_command(callback=None, msg_defn=msg)

    msg = MsgGpioExpanderControl().set_param("Module", handle).set_param("Channel", 0).set_param("OutputValue", 0)
    resp = yasp_client.send_command(callback=None, msg_defn=msg)


adc_data = open("adc_data.csv", "w")


def cb_adc_data(msg_defn):
    data = msg_defn.get_param("AnalogData")
    unpacked = struct.unpack("<%dh" % (len(data) / 2), data)
    data_list = [i for i in unpacked]
    for i in data_list:
        adc_data.write("%d,\n" % i)


def test_adc_stream(yasp_client):
    msg = MsgAnalogDataStream().set_param("Instance", 0).set_param("Channel", 0).set_param("DataRate", 2)\
        .set_param("DataRange", 3).set_param("Watermark", 1)
    resp = yasp_client.send_command(callback=None, msg_defn=msg)
    time.sleep(30)


def adc_sample_cb(msg):
    data = struct.unpack("<h", msg.get_param("AdcData"))
    instance = msg.get_param("Instance")
    print("ADC Data: %d-%d" % (instance, data[0]))


def test_adc_sample(yasp_client):
    msg = MsgPinFunction().set_param("Module", 0).set_param("PinIndex", 0)\
        .set_param("Instance", 0).set_param("Function", 16).set_param("SaveConfig", 1)
    resp = yasp_client.send_command(callback=None, msg_defn=msg)
    resp = yasp_client.send_command(callback=None, msg_defn=msg.set_param("Instance", 1))
    time.sleep(1)
    msg = MsgAnalogInput().set_param("Instance", 0).set_param("DataRate", 10)\
        .set_param("DataRange", 0).set_param("Watermark", 0)
    resp = yasp_client.send_command(callback=None, msg_defn=msg)
    resp = yasp_client.send_command(callback=None, msg_defn=msg.set_param("Instance", 1))
    time.sleep(10)
    resp = yasp_client.send_command(callback=None, msg_defn=msg.set_param("DataRate", 0))
    resp = yasp_client.send_command(callback=None, msg_defn=msg.set_param("Instance", 0))


def pin_control_cb(msg):
    print("Pin Control Name: %s" % (msg.get_param("PinControlName")))


def test_pin_control(yasp_client):
    msg = MsgPinFunction().set_param("Module", 2).set_param("PinIndex", 1)\
        .set_param("Function", MsgPinFunction.FUNC_CONTROL).set_param("Instance", 1)\
        .set_param("SaveConfig", 1)
    resp = yasp_client.send_command(callback=None, msg_defn=msg)
    msg = MsgPinControl().set_param("Instance", 1).set_param("SaveConfig", 2)\
        .set_param("DriveConfig", 0).set_param("OutputState", 0).set_param("PinControlName", "TestName")
    resp = yasp_client.send_command(callback=pin_control_cb, msg_defn=msg)
    time.sleep(1)


def test_read_pin_control(yasp_client):
    msg = MsgPinControl().set_param("Instance", 1).set_param("SaveConfig", 0)\
        .set_param("DriveConfig", 0).set_param("OutputState", 0).set_param("PinControlName", "")
    resp = yasp_client.send_command(callback=pin_control_cb, msg_defn=msg)
    time.sleep(1)


def pin_monitor_cb(msg):
    print("Pin Monitor Name: %s" % (msg.get_param("PinMonitorName")))


def test_pin_monitor(yasp_client):
    msg = MsgPinFunction().set_param("Module", 2).set_param("PinIndex", 2) \
        .set_param("Function", MsgPinFunction.FUNC_MONITOR).set_param("Instance", 1) \
        .set_param("SaveConfig", 1)
    resp = yasp_client.send_command(callback=None, msg_defn=msg)
    msg = MsgPinMonitor().set_param("Instance", 1).set_param("SaveConfig", 2).set_param("PullConfig", 0)\
        .set_param("SenseConfig", 0).set_param("PinMonitorName", "TestMon")
    resp = yasp_client.send_command(callback=pin_monitor_cb, msg_defn=msg)
    time.sleep(1)


def test_read_pin_monitor(yasp_client):
    msg = MsgPinMonitor().set_param("Instance", 1).set_param("SaveConfig", 0).set_param("PinMonitorName", "")
    resp = yasp_client.send_command(callback=pin_monitor_cb, msg_defn=msg)
    time.sleep(1)


def main():
    client = YaspBlueGigaClient(port="COM7")
    client.reset_ble_state()
    client.pipe_logs_to_terminal(level=logging.INFO)
    yasp_client = YaspClient()
    conn_handle = client.connect_by_name(DEVICE_NAME, yasp_client)
    #conn_handle = client.connect_by_name("Bluemo v2.0", yasp_client)
    yasp_client.set_default_msg_callback(MsgError.get_response_code(), cb_error)
    yasp_client.set_default_msg_callback(MsgPinFunction.get_response_code(), pin_function_cb)
    yasp_client.set_default_msg_callback(MsgBatteryStatus.get_response_code(), cb_battery_control)
    yasp_client.set_default_msg_callback(MsgAccelStream.get_response_code(), cb_stream_accelerometer)
    yasp_client.set_default_msg_callback(MsgGyroStream.get_response_code(), cb_gyro_stream)
    yasp_client.set_default_msg_callback(MsgAnalogDataStream.get_response_code(), cb_adc_data)
    yasp_client.set_default_msg_callback(MsgServoConfig.get_response_code(), servo_config_cb)
    yasp_client.set_default_msg_callback(MsgAnalogInput.get_response_code(), adc_sample_cb)
    yasp_client.set_default_msg_callback(MsgPinMonitor.get_response_code(), pin_monitor_cb)
    #test_device_config(yasp_client)
    #test_soft_reset(client, yasp_client)
    #test_loopback(yasp_client)
    test_detect_module(yasp_client)
    #test_pin_function(yasp_client)
    #test_ws2812(yasp_client)
    #test_adc_stream(yasp_client)
    #test_adc_sample(yasp_client)
    #test_pin_monitor(yasp_client)
    #test_pin_control(yasp_client)
    #test_read_pin_control(yasp_client)
    #test_read_pin_monitor(yasp_client)
    time.sleep(1.1)
    client.disconnect(conn_handle)


if __name__ == "__main__":
    main()
