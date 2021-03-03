from pybluemo.bluemo_spec import *
import struct


class MessageDefinition(object):
    _MSG_NAME = None

    def __init__(self):
        self.parameters = {}

    def _pack(self, param_list):
        if len(param_list) == 0:
            return ""
        format_str = "<"
        params_tuple = tuple()
        for param in param_list:
            if param[MSG_PARAM_TYPE] == "uint8_t":
                format_str += "B"
            elif param[MSG_PARAM_TYPE] == "uint16_t":
                format_str += "H"
            elif param[MSG_PARAM_TYPE] == "int16_t":
                format_str += "h"
            elif param[MSG_PARAM_TYPE] == "uint32_t":
                format_str += "I"
            elif param[MSG_PARAM_TYPE] == "int32_t":
                format_str += "i"
            elif param[MSG_PARAM_NAME] == "float":
                format_str += "f"
            elif param[MSG_PARAM_TYPE] == "uint8_t *":
                format_str += "%ds" % len(self.get_param(param[MSG_PARAM_NAME]))
            if param[MSG_PARAM_NAME] in self.parameters:
                params_tuple += (self.get_param(param[MSG_PARAM_NAME]),)
            elif MSG_PARAM_DEFAULT in param:
                params_tuple += (param[MSG_PARAM_DEFAULT], )
            else:
                raise Exception("Required parameter not specified: %s" % param[MSG_PARAM_NAME])
        return struct.pack(format_str, *params_tuple)

    def _unpack(self, payload, param_list):
        format_str = "<"
        offset = 0
        for param in param_list:
            if param[MSG_PARAM_TYPE] == "uint8_t":
                format_str += "B"
                offset += 1
            elif param[MSG_PARAM_TYPE] == "uint16_t":
                format_str += "H"
                offset += 2
            elif param[MSG_PARAM_TYPE] == "int16_t":
                format_str += "h"
                offset += 2
            elif param[MSG_PARAM_TYPE] == "uint32_t":
                format_str += "I"
                offset += 4
            elif param[MSG_PARAM_TYPE] == "int32_t":
                format_str += "i"
                offset += 4
            elif param[MSG_PARAM_TYPE] == "float":
                format_str += "f"
                offset += 4
            elif param[MSG_PARAM_TYPE] == "uint8_t *":
                format_str += "%ds" % (len(payload) - offset)
        params_tuple = struct.unpack(format_str, payload)
        for i in range(len(param_list)):
            self.set_param(param_list[i][MSG_PARAM_NAME], params_tuple[i])

    def set_param(self, name, value):
        self.parameters[name] = value
        return self

    def get_param(self, name):
        return self.parameters[name]

    def cmd_msg(self):
        return self._pack(self.get_command_params())

    @classmethod
    def cmd_recv(cls, payload):
        inst = cls()
        inst._unpack(payload, inst.get_command_params())
        return inst

    def rsp_msg(self):
        return self._pack(self.get_response_params())

    @classmethod
    def rsp_recv(cls, payload):
        inst = cls()
        inst._unpack(payload, inst.get_response_params())
        return inst

    def get_command_params(self):
        return MESSAGES_BY_NAME[self._MSG_NAME][MSG_COMMAND]

    def get_response_params(self):
        return MESSAGES_BY_NAME[self._MSG_NAME][MSG_RESPONSE]

    @classmethod
    def get_command_code(cls):
        return MESSAGES_BY_NAME[cls._MSG_NAME][MSG_CODE]

    @classmethod
    def get_response_code(cls):
        return cls.get_command_code() + 128


class MsgError(MessageDefinition):
    _MSG_NAME = "Error"


class MsgDeviceConfig(MessageDefinition):
    _MSG_NAME = "DeviceConfig"


class MsgSoftReset(MessageDefinition):
    _MSG_NAME = "SoftReset"


class MsgLoopback(MessageDefinition):
    _MSG_NAME = "Loopback"


class MsgBatteryStatus(MessageDefinition):
    _MSG_NAME = "BatteryStatus"

    def get_battery_voltage(self):
        return 2.304 + self.get_param("BatteryVoltage") * 0.02

    def get_system_voltage(self):
        return 2.304 + self.get_param("SystemVoltage") * 0.02

    def get_usb_voltage(self):
        return 2.6 + self.get_param("UsbBusVoltage") * 0.1

    def get_charge_current(self):
        return 0.0 + self.get_param("ChargeCurrent") * 0.05


class MsgDetectModule(MessageDefinition):
    _MSG_NAME = "DetectModule"


class MsgFlashErase(MessageDefinition):
    _MSG_NAME = "FlashErase"


class MsgFlashWrite(MessageDefinition):
    _MSG_NAME = "FlashWrite"


class MsgFlashRead(MessageDefinition):
    _MSG_NAME = "FlashRead"


class MsgFlashInitialize(MessageDefinition):
    _MSG_NAME = "FlashInit"


class MsgFlashDisconnect(MessageDefinition):
    _MSG_NAME = "FlashDisconnect"


class MsgTwimScan(MessageDefinition):
    _MSG_NAME = "TwimScan"


class MsgTwimTransfer(MessageDefinition):
    _MSG_NAME = "TwimTransfer"


class MsgLedConfig(MessageDefinition):
    _MSG_NAME = "LedConfig"
    FRAME_DATA = "FrameData"

    ANIM_FIXED = 0
    ANIM_ROTATION = 1
    ANIM_ROT_INDIVIDUAL = 2
    ANIM_FRAME_BY_FRAME = 3
    ANIM_GRAVITY_X = 4

    TYPE_NORMAL = 0
    TYPE_ASCENDING = 1
    TYPE_DESCENDING = 2
    TYPE_UPDOWN = 3
    TYPE_REV_ASC = 4
    TYPE_REV_DESC = 5
    TYPE_REV_UPDOWN = 6

    def add_run_length(self, offset, length, color, horiz=1, rle_type=TYPE_NORMAL, eof=False):
        payload = ""
        if eof:
            eof = 0x80
        else:
            eof = 0x00
        payload += chr((((offset >> 6) & 0x0F) + eof) + (rle_type << 4))
        payload += chr(((offset << 2) & 0xFC) + ((length >> 8) & 0x03))
        payload += chr(length & 0xFF)
        payload += color
        payload += chr(horiz & 0xFF)
        if self.FRAME_DATA in self.parameters:
            self.parameters[self.FRAME_DATA] += payload
        else:
            self.parameters[self.FRAME_DATA] = payload
        #print("".join(["%02X" % ord(i) for i in self.parameters[self.FRAME_DATA]]))
        return self


class MsgLedPattern(MsgLedConfig):
    _MSG_NAME = "LedPattern"


class MsgPinFunction(MessageDefinition):
    _MSG_NAME = "PinFunction"
    FUNC_NONE = 0
    FUNC_CONTROL = 1
    FUNC_MONITOR = 2
    FUNC_ANALOG = 3
    FUNC_WS2812 = 4
    FUNC_SERVO_PWM = 5
    FUNC_SERVO_CLK = 6
    FUNC_SERVO_RST = 7
    FUNC_MOTOR_CHAN = 8
    FUNC_MOTOR_ADDR = 9
    FUNC_GPIO_INT = 10
    FUNC_GPIO_ADDR = 11
    FUNC_GPIO_LED_LSB = 12
    FUNC_GPIO_LED_MSB = 13
    FUNC_ROBOTIS_ADDR = 14
    FUNC_ROBOTIS_PWR = 15
    FUNC_ROBOTIS_DATA = 16
    FUNC_ANALOG_INT = 17
    FUNC_ANALOG_ADDR = 18


class MsgPinMonitor(MessageDefinition):
    _MSG_NAME = "PinMonitor"
    PULL_NONE = 0
    PULL_UP = 1
    PULL_DOWN = 2

    SENSE_NONE = 0
    SENSE_LOW = 1
    SENSE_HIGH = 2
    SENSE_BOTH = 3


class MsgPinControl(MessageDefinition):
    _MSG_NAME = "PinControl"
    DRIVE_STD = 0


class MsgRobotisConfig(MessageDefinition):
    _MSG_NAME = "RobotisConfig"


class MsgRobotisCommand(MessageDefinition):
    _MSG_NAME = "RobotisCommand"


class MsgServoConfig(MessageDefinition):
    _MSG_NAME = "ServoConfig"


class MsgServoControl(MessageDefinition):
    _MSG_NAME = "ServoControl"


class MsgAccelStream(MessageDefinition):
    _MSG_NAME = "AccelStream"


class MsgGyroStream(MessageDefinition):
    _MSG_NAME = "GyroStream"


class MsgMotorConfig(MessageDefinition):
    _MSG_NAME = "MotorConfig"


class MsgMotorControl(MessageDefinition):
    _MSG_NAME = "MotorControl"


class MsgGpioExpanderConfig(MessageDefinition):
    _MSG_NAME = "GpioExpanderConfig"


class MsgGpioExpanderControl(MessageDefinition):
    _MSG_NAME = "GpioExpanderControl"


class MsgAnalogDataStream(MessageDefinition):
    _MSG_NAME = "AnalogDataStream"


class MsgResourceConfiguration(MessageDefinition):
    _MSG_NAME = "ResourceConfiguration"


class MsgStartOrientation(MessageDefinition):
    _MSG_NAME = "StartOrientation"


class MsgAnalogInput(MessageDefinition):
    _MSG_NAME = "AnalogInput"


MSG_CLASSES = [MsgError, MsgDeviceConfig, MsgSoftReset, MsgLoopback, MsgBatteryStatus, MsgDetectModule,
               MsgFlashErase, MsgFlashWrite, MsgFlashRead, MsgFlashInitialize, MsgFlashDisconnect,
               MsgTwimScan, MsgTwimTransfer, MsgLedConfig, MsgLedPattern, MsgPinFunction, MsgPinControl, MsgPinMonitor,
               MsgRobotisConfig, MsgRobotisCommand, MsgServoConfig, MsgServoControl,
               MsgAccelStream, MsgGyroStream, MsgMotorConfig, MsgMotorControl, MsgGpioExpanderConfig,
               MsgGpioExpanderControl, MsgAnalogDataStream, MsgResourceConfiguration, MsgStartOrientation,
               MsgAnalogInput]

MSG_CLASS_BY_CMD_CODE = {}
MSG_CLASS_BY_RSP_CODE = {}

for _MSG in MSG_CLASSES:
    MSG_CLASS_BY_CMD_CODE[_MSG.get_command_code()] = _MSG
    MSG_CLASS_BY_RSP_CODE[_MSG.get_response_code()] = _MSG
