import struct


MSGS = "Messages"
MSG_NAME = "Name"
MSG_COMMAND = "Command"
MSG_CODE = "Command Code"
MSG_RESPONSE = "Response"
MSG_DESC = "Description"

MSG_PARAM_NAME = "Name"
MSG_PARAM_TYPE = "Type"
MSG_PARAM_DESC = "Description"
MSG_PARAM_DEFAULT = "Default"


class MessageDefinition(object):
    _MSG_NAME = None
    _CMD_PARAMS = None
    _RSP_PARAMS = None
    _CMD_CODE = None

    def __init__(self):
        self.parameters = {}

    def __str__(self):
        return self._MSG_NAME + " " + " - ".join(["%s:%s" % (param, self.parameters[param]) for param in self.parameters])

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
        return self._CMD_PARAMS

    def get_response_params(self):
        return self._RSP_PARAMS

    @classmethod
    def get_command_code(cls):
        return cls._CMD_CODE

    @classmethod
    def get_response_code(cls):
        return cls.get_command_code() + 128


# --------------------------- BEGIN GENERATED CODE ------------------------- #


class EnumModify(object):
    READ_ONLY = 0
    MODIFY = 1
    SAVE_TO_FLASH = 2


class EnumBatteryType(object):
    NO_BATTERY = 0
    LITHIUM3_P7_V = 1


class EnumDriveConfig(object):
    DISCONNECT = 0
    INPUT_NO_PULL = 1
    INPUT_PULL_DOWN = 2
    INPUT_PULL_UP = 3
    INPUT_PULL_UP_WITH_SENSING = 4
    INPUT_NO_PULL_WITH_SENSING = 5
    OUTPUT_LOW = 6
    OUTPUT_HIGH = 7


class EnumI2cFreq(object):
    F100K_HZ = 0
    F250K_HZ = 1
    F400K_HZ = 2
    TOTAL_FREQS = 3


class EnumSpiFreq(object):
    F125KBPS = 0
    F250KBPS = 1
    F500KBPS = 2
    F1_MBPS = 3
    F2_MBPS = 4
    F4_MBPS = 5
    TOTAL_FREQS = 6


class EnumPullConfig(object):
    NO_PULL = 0
    PULL_UP = 1
    PULL_DOWN = 2


class EnumSenseConfig(object):
    NONE = 0
    SENSE_LOW = 1
    SENSE_HIGH = 2
    SENSE_BOTH = 3


class EnumPinFunction(object):
    DISCONNECT = 0
    PIN_CONTROL = 1
    PIN_MONITOR = 2
    PIN_ANALOG_INPUT = 3
    PIN_LED_DATA = 4
    PIN_SERVO_PULSE = 5
    PIN_SERVO_CLOCK = 6
    PIN_SERVO_RESET = 7
    PIN_PWM_CHANNEL = 8
    PIN_ROBOTIS = 9
    TOTAL_FUNCTIONS = 10


class EnumDriveStrength(object):
    S0_S1 = 0
    H0_S1 = 1
    S0_H1 = 2
    H0_H1 = 3
    D0_S1 = 4
    D0_H1 = 5
    S0_D1 = 6
    H0_D1 = 7


class EnumSensorModel(object):
    BMA280 = 0
    BMA400 = 1
    BMG250 = 2


class EnumFlashModel(object):
    UNKNOWN_DEVICE_ID = 0
    AT25_DF321_A = 1
    AT45_DB641_E = 2


class EnumAccelDataRange(object):
    G4 = 0
    G8 = 1
    G16 = 2
    G32 = 3


class EnumAccelDataRate(object):
    OFF = 0
    F7P81 = 1
    F15P63 = 2
    F31P25 = 3
    F62P5 = 4
    F125 = 5
    F250 = 6
    F500 = 7
    F12P5 = 8
    F25 = 9
    F50 = 10
    F100 = 11
    F200 = 12
    F400 = 13
    F800 = 14


class EnumSensorEvent(object):
    SINGLE_TAP = 0
    DOUBLE_TAP = 1
    ORIENTATION = 2
    FREE_FALL = 3
    NO_MOTION = 4
    STEP_DETECT = 5
    DOUBLE_STEP = 6
    ACTIVITY_CHANGE_X = 7
    ACTIVITY_CHANGE_Y = 8
    ACTIVITY_CHANGE_Z = 9


class EnumGyroDataRange(object):
    PM2000_D = 0
    PM1000_D = 1
    PM500_D = 2
    PM250_D = 3
    PM125_D = 4


class EnumGyroDataRate(object):
    OFF = 0
    F25 = 1
    F50 = 2
    F100 = 3
    F200 = 4
    F400 = 5
    F800 = 6
    F1600 = 7
    F3200 = 8


class EnumSaadcPullConfig(object):
    NO_PULL = 0
    PULL_DOWN160K = 1
    PULL_UP160K = 2


class EnumSaadcGain(object):
    ONE_SIXTH = 0
    ONE_FIFTH = 1
    ONE_FOURTH = 2
    ONE_THIRD = 3
    ONE_HALF = 4
    UNITY = 5
    DOUBLE = 6
    QUADRUPLE = 7


class EnumSaadcRefSel(object):
    INTERN0P6_V = 0
    VDD_DIV4 = 1


class EnumSaadcTacq(object):
    THREE_MICRO = 0
    FIVE_MICRO = 1
    TEN_MICRO = 2
    FIFTEEN_MICRO = 3
    TWENTY_MICRO = 4
    FORTY_MICRO = 5


class EnumPhyCoding(object):
    ONE_MBPS = 0
    TWO_MBPS = 1
    FIVE_HUNDRED_KBPS = 2
    ONE_TWO_FIVE_KBPS = 3


class EnumTxPower(object):
    NEG40D_BM = 0
    NEG20D_BM = 1
    NEG16D_BM = 2
    NEG12D_BM = 3
    NEG8D_BM = 4
    NEG4D_BM = 5
    ZEROD_BM = 6
    POS2D_BM = 7
    POS3D_BM = 8
    POS4D_BM = 9
    POS5D_BM = 10
    POS6D_BM = 11
    POS7D_BM = 12
    POS8D_BM = 13


class EnumAdvertiseIn(object):
    DO_NOT_INCLUDE = 0
    ADVERTISEMENT = 1
    SCAN_RESPONSE = 2


class EnumAdvertisingType(object):
    CONNECTABLE_SCANNABLE_UNDIRECTED = 0
    CONNECTABLE_NONSCANNABLE_DIRECTED_FAST = 1
    CONNECTABLE_NONSCANNABLE_DIRECTED = 2
    EXTENDED_CONNECTABLE_NONSCANNABLE_UNDIRECTED = 3
    EXTENDED_CONNECTABLE_NONSCANNABLE_DIRECTED = 4


class EnumAnalogInput(object):
    A_I_N0 = 0
    A_I_N1 = 1
    A_I_N2 = 2
    A_I_N3 = 3
    A_I_N4 = 4
    A_I_N5 = 5
    A_I_N6 = 6
    A_I_N7 = 7
    V_D_D_H_D_I_V5 = 8


class EnumFilterType(object):
    NO_FILTER = 0
    BY_ADDRESS = 1
    BY_ADV_DATA = 2
    BONDED_ONLY = 3


class EnumActionType(object):
    DISCONNECT = 0
    BLUEMO_INIT = 1


class EnumConnectionStatus(object):
    NOT_CONNECTED = 0
    PERIPHERAL_ROLE = 1
    CENTRAL_ROLE = 2


class EnumAnimationType(object):
    NONE = 0
    ROTATION = 1
    SEGMENT_ROTATION = 2
    FRAME_BY_FRAME = 3


class EnumPatternType(object):
    NORMAL = 0
    ASCENDING = 1
    DESCENDING = 2
    UP_DOWN = 3
    REVERSE_ASCENDING = 4
    REVERSE_DESCENDING = 5
    REVERSE_UP_DOWN = 6


class EnumRgbColorBytes(object):
    RED_GREEN_BLUE = 0
    GREEN_RED_BLUE = 1
    RED_GREEN_BLUE_INVERTED = 2
    GREEN_RED_BLUE_INVERTED = 3


class MsgError(MessageDefinition):
    _MSG_NAME = "Error"
    _CMD_CODE = 0
    _CMD_PARAMS = [{'Name': 'ErrorMessage', 'Type': 'uint8_t *'}]
    _RSP_PARAMS = [{'Name': 'ErrorMessage', 'Type': 'uint8_t *'}]

    @classmethod
    def builder(cls, error_message):
        msg = cls()
        msg.parameters = {
            "ErrorMessage": error_message,
        }
        return msg


class MsgDeviceName(MessageDefinition):
    _MSG_NAME = "DeviceName"
    _CMD_CODE = 1
    _CMD_PARAMS = [{'Name': 'Modify', 'Type': 'uint8_t', 'Enum': 'Modify', 'Default': 0}, {'Name': 'DeviceName', 'Type': 'uint8_t *', 'MaxLength': 24}]
    _RSP_PARAMS = [{'Name': 'DeviceName', 'Type': 'uint8_t *'}]

    @classmethod
    def builder(cls, device_name, modify=0):
        msg = cls()
        msg.parameters = {
            "Modify": modify,
            "DeviceName": device_name,
        }
        return msg


class MsgSoftReset(MessageDefinition):
    _MSG_NAME = "SoftReset"
    _CMD_CODE = 2
    _CMD_PARAMS = []
    _RSP_PARAMS = []

    @classmethod
    def builder(cls):
        msg = cls()
        msg.parameters = {
        }
        return msg


class MsgMfrOtp(MessageDefinition):
    _MSG_NAME = "MfrOtp"
    _CMD_CODE = 3
    _CMD_PARAMS = [{'Name': 'Modify', 'Type': 'uint8_t', 'Enum': 'Modify'}, {'Name': 'SerialNumber', 'Type': 'uint32_t'}, {'Name': 'ModelNumber', 'Type': 'uint32_t'}, {'Name': 'HardwareRevision', 'Type': 'uint8_t *', 'MaxLength': 28}]
    _RSP_PARAMS = [{'Name': 'SerialNumber', 'Type': 'uint32_t'}, {'Name': 'ModelNumber', 'Type': 'uint32_t'}, {'Name': 'HardwareRevision', 'Type': 'uint8_t *', 'MaxLength': 28}]

    @classmethod
    def builder(cls, modify, serial_number, model_number, hardware_revision):
        msg = cls()
        msg.parameters = {
            "Modify": modify,
            "SerialNumber": serial_number,
            "ModelNumber": model_number,
            "HardwareRevision": hardware_revision,
        }
        return msg


class MsgLoopback(MessageDefinition):
    _MSG_NAME = "Loopback"
    _CMD_CODE = 4
    _CMD_PARAMS = [{'Name': 'Periodicity', 'Type': 'uint8_t', 'Range': 'LoopbackPeriodicity'}, {'Name': 'Payload', 'Type': 'uint8_t *', 'MaxLength': 256}]
    _RSP_PARAMS = [{'Name': 'Periodicity', 'Type': 'uint8_t'}, {'Name': 'Payload', 'Type': 'uint8_t *'}]

    @classmethod
    def builder(cls, periodicity, payload):
        msg = cls()
        msg.parameters = {
            "Periodicity": periodicity,
            "Payload": payload,
        }
        return msg


class MsgPinConfig(MessageDefinition):
    _MSG_NAME = "PinConfig"
    _CMD_CODE = 5
    _CMD_PARAMS = [{'Name': 'Modify', 'Type': 'uint8_t', 'Enum': 'Modify', 'Default': 0}, {'Name': 'PinIndex', 'Type': 'uint8_t', 'Range': 'PinIndex', 'Index': True}, {'Name': 'Function', 'Type': 'uint8_t', 'Enum': 'PinFunction', 'Default': 0}, {'Name': 'Instance', 'Type': 'int8_t', 'Range': 'OptionalInstance', 'Default': -1}, {'Name': 'PinName', 'Type': 'uint8_t *', 'MaxLength': 24}]
    _RSP_PARAMS = [{'Name': 'PinIndex', 'Type': 'uint8_t', 'Index': True}, {'Name': 'Function', 'Type': 'uint8_t', 'Enum': 'PinFunction'}, {'Name': 'Instance', 'Type': 'uint8_t'}, {'Name': 'PinName', 'Type': 'uint8_t *'}]

    @classmethod
    def builder(cls, pin_index, pin_name, modify=0, function=0, instance=-1):
        msg = cls()
        msg.parameters = {
            "Modify": modify,
            "PinIndex": pin_index,
            "Function": function,
            "Instance": instance,
            "PinName": pin_name,
        }
        return msg


class MsgPinControl(MessageDefinition):
    _MSG_NAME = "PinControl"
    _CMD_CODE = 6
    _CMD_PARAMS = [{'Name': 'Instance', 'Type': 'uint8_t', 'Range': 'PinControlInstance', 'Index': True}, {'Name': 'Modify', 'Type': 'uint8_t', 'Enum': 'Modify', 'Default': 0}, {'Name': 'DriveStrength', 'Type': 'uint8_t', 'Enum': 'DriveStrength', 'Default': 0}, {'Name': 'OutputState', 'Type': 'uint8_t', 'Range': 'Boolean', 'Default': 0}]
    _RSP_PARAMS = [{'Name': 'Instance', 'Type': 'uint8_t', 'Index': True}, {'Name': 'DriveStrength', 'Type': 'uint8_t'}, {'Name': 'OutputState', 'Type': 'uint8_t'}]

    @classmethod
    def builder(cls, instance, modify=0, drive_strength=0, output_state=0):
        msg = cls()
        msg.parameters = {
            "Instance": instance,
            "Modify": modify,
            "DriveStrength": drive_strength,
            "OutputState": output_state,
        }
        return msg


class MsgPinMonitor(MessageDefinition):
    _MSG_NAME = "PinMonitor"
    _CMD_CODE = 7
    _CMD_PARAMS = [{'Name': 'Instance', 'Type': 'uint8_t', 'Range': 'PinMonitorInstance', 'Index': True}, {'Name': 'Modify', 'Type': 'uint8_t', 'Enum': 'Modify', 'Default': 0}, {'Name': 'PullConfig', 'Type': 'uint8_t', 'Enum': 'PullConfig', 'Default': 0}, {'Name': 'SenseConfig', 'Type': 'uint8_t', 'Enum': 'SenseConfig', 'Default': 0}]
    _RSP_PARAMS = [{'Name': 'Instance', 'Type': 'uint8_t', 'Index': True}, {'Name': 'PullConfig', 'Type': 'uint8_t', 'Enum': 'PullConfig'}, {'Name': 'SenseConfig', 'Type': 'uint8_t', 'Enum': 'SenseConfig'}, {'Name': 'PinStatus', 'Type': 'uint8_t'}]

    @classmethod
    def builder(cls, instance, modify=0, pull_config=0, sense_config=0):
        msg = cls()
        msg.parameters = {
            "Instance": instance,
            "Modify": modify,
            "PullConfig": pull_config,
            "SenseConfig": sense_config,
        }
        return msg


class MsgTwimInit(MessageDefinition):
    _MSG_NAME = "TwimInit"
    _CMD_CODE = 8
    _CMD_PARAMS = [{'Name': 'Modify', 'Type': 'uint8_t', 'Enum': 'Modify', 'Default': 0}, {'Name': 'SclPin', 'Type': 'int8_t', 'Range': 'OptionalPinIndex', 'Default': -1}, {'Name': 'SdaPin', 'Type': 'int8_t', 'Range': 'OptionalPinIndex', 'Default': -1}, {'Name': 'Frequency', 'Type': 'uint8_t', 'Enum': 'I2cFreq', 'Default': 2}]
    _RSP_PARAMS = [{'Name': 'SclPin', 'Type': 'uint8_t'}, {'Name': 'SdaPin', 'Type': 'uint8_t'}, {'Name': 'Frequency', 'Type': 'uint8_t', 'Enum': 'I2cFreq'}]

    @classmethod
    def builder(cls, modify=0, scl_pin=-1, sda_pin=-1, frequency=2):
        msg = cls()
        msg.parameters = {
            "Modify": modify,
            "SclPin": scl_pin,
            "SdaPin": sda_pin,
            "Frequency": frequency,
        }
        return msg


class MsgTwimDisconnect(MessageDefinition):
    _MSG_NAME = "TwimDisconnect"
    _CMD_CODE = 9
    _CMD_PARAMS = []
    _RSP_PARAMS = []

    @classmethod
    def builder(cls):
        msg = cls()
        msg.parameters = {
        }
        return msg


class MsgTwimScan(MessageDefinition):
    _MSG_NAME = "TwimScan"
    _CMD_CODE = 10
    _CMD_PARAMS = [{'Name': 'TestByte', 'Type': 'uint8_t', 'Description': 'Byte of data used to attempt an I2C write.'}]
    _RSP_PARAMS = [{'Name': 'SlaveAddress', 'Type': 'uint8_t'}, {'Name': 'TotalFound', 'Type': 'uint8_t'}]

    @classmethod
    def builder(cls, test_byte):
        msg = cls()
        msg.parameters = {
            "TestByte": test_byte,
        }
        return msg


class MsgTwimTransfer(MessageDefinition):
    _MSG_NAME = "TwimTransfer"
    _CMD_CODE = 11
    _CMD_PARAMS = [{'Name': 'Handle', 'Type': 'uint8_t', 'Description': 'Software specified operation handle copied by firmware to the response message', 'Default': 0}, {'Name': 'SlaveAddress', 'Type': 'uint8_t', 'Range': 'TwimAddress'}, {'Name': 'ReadLength', 'Type': 'uint16_t', 'Range': 'BufferLimited'}, {'Name': 'WriteData', 'Type': 'uint8_t *'}]
    _RSP_PARAMS = [{'Name': 'Handle', 'Type': 'uint8_t', 'Description': 'Copy of Handle provided in command packet, not relevant to firmware, but this can be used to identify responses in software when multiple concurrent transfers are in progress.'}, {'Name': 'ActualWriteLength', 'Type': 'uint16_t', 'Description': 'Actual size written, if a write was attempted but no I2C ack is received, this will be zero.'}, {'Name': 'ActualReadData', 'Type': 'uint8_t *', 'Description': 'Actual size read, if a read was attempted but no I2C ack is received, this will be zero.'}]

    @classmethod
    def builder(cls, slave_address, read_length, write_data, handle=0):
        msg = cls()
        msg.parameters = {
            "Handle": handle,
            "SlaveAddress": slave_address,
            "ReadLength": read_length,
            "WriteData": write_data,
        }
        return msg


class MsgSpiInit(MessageDefinition):
    _MSG_NAME = "SpiInit"
    _CMD_CODE = 12
    _CMD_PARAMS = [{'Name': 'Modify', 'Type': 'uint8_t', 'Enum': 'Modify', 'Default': 0}, {'Name': 'ClkPin', 'Type': 'int8_t', 'Range': 'OptionalPinIndex', 'Default': -1}, {'Name': 'MosiPin', 'Type': 'int8_t', 'Range': 'OptionalPinIndex', 'Default': -1}, {'Name': 'MisoPin', 'Type': 'int8_t', 'Range': 'OptionalPinIndex', 'Default': -1}, {'Name': 'Frequency', 'Type': 'uint8_t', 'Enum': 'SpiFreq', 'Default': 3}]
    _RSP_PARAMS = [{'Name': 'ClkPin', 'Type': 'uint8_t'}, {'Name': 'MosiPin', 'Type': 'uint8_t'}, {'Name': 'MisoPin', 'Type': 'uint8_t'}, {'Name': 'Frequency', 'Type': 'uint8_t', 'Enum': 'SpiFreq'}]

    @classmethod
    def builder(cls, modify=0, clk_pin=-1, mosi_pin=-1, miso_pin=-1, frequency=3):
        msg = cls()
        msg.parameters = {
            "Modify": modify,
            "ClkPin": clk_pin,
            "MosiPin": mosi_pin,
            "MisoPin": miso_pin,
            "Frequency": frequency,
        }
        return msg


class MsgSpiDisconnect(MessageDefinition):
    _MSG_NAME = "SpiDisconnect"
    _CMD_CODE = 13
    _CMD_PARAMS = []
    _RSP_PARAMS = []

    @classmethod
    def builder(cls):
        msg = cls()
        msg.parameters = {
        }
        return msg


class MsgSpiTransfer(MessageDefinition):
    _MSG_NAME = "SpiTransfer"
    _CMD_CODE = 14
    _CMD_PARAMS = [{'Name': 'Handle', 'Type': 'uint8_t'}, {'Name': 'CsPinControlInstance', 'Type': 'uint8_t', 'Range': 'PinControlInstance'}, {'Name': 'ReadLength', 'Type': 'uint16_t', 'Range': 'BufferLimited'}, {'Name': 'WriteData', 'Type': 'uint8_t *'}]
    _RSP_PARAMS = [{'Name': 'Handle', 'Type': 'uint8_t'}, {'Name': 'CsPinControlInstance', 'Type': 'uint8_t'}, {'Name': 'WriteLength', 'Type': 'uint16_t'}, {'Name': 'ReadData', 'Type': 'uint8_t *'}]

    @classmethod
    def builder(cls, handle, cs_pin_control_instance, read_length, write_data):
        msg = cls()
        msg.parameters = {
            "Handle": handle,
            "CsPinControlInstance": cs_pin_control_instance,
            "ReadLength": read_length,
            "WriteData": write_data,
        }
        return msg


class MsgSpiFlashInit(MessageDefinition):
    _MSG_NAME = "SpiFlashInit"
    _CMD_CODE = 15
    _CMD_PARAMS = [{'Name': 'Modify', 'Type': 'uint8_t', 'Enum': 'Modify', 'Default': 0}, {'Name': 'ChipSelectPin', 'Type': 'int8_t', 'Range': 'OptionalPinIndex', 'Default': -1}, {'Name': 'WriteProtectPin', 'Type': 'int8_t', 'Range': 'OptionalPinIndex', 'Default': -1}, {'Name': 'ResetPin', 'Type': 'int8_t', 'Range': 'OptionalPinIndex', 'Default': -1}]
    _RSP_PARAMS = [{'Name': 'FlashModel', 'Type': 'uint8_t', 'Enum': 'FlashModel'}, {'Name': 'CsPinControlInstance', 'Type': 'uint8_t'}, {'Name': 'WpPinControlInstance', 'Type': 'uint8_t'}, {'Name': 'RstPinControlInstance', 'Type': 'uint8_t'}, {'Name': 'DeviceId', 'Type': 'uint8_t *'}]

    @classmethod
    def builder(cls, modify=0, chip_select_pin=-1, write_protect_pin=-1, reset_pin=-1):
        msg = cls()
        msg.parameters = {
            "Modify": modify,
            "ChipSelectPin": chip_select_pin,
            "WriteProtectPin": write_protect_pin,
            "ResetPin": reset_pin,
        }
        return msg


class MsgSpiFlashDisconnect(MessageDefinition):
    _MSG_NAME = "SpiFlashDisconnect"
    _CMD_CODE = 16
    _CMD_PARAMS = []
    _RSP_PARAMS = []

    @classmethod
    def builder(cls):
        msg = cls()
        msg.parameters = {
        }
        return msg


class MsgSpiFlashErase(MessageDefinition):
    _MSG_NAME = "SpiFlashErase"
    _CMD_CODE = 17
    _CMD_PARAMS = [{'Name': 'CsPinControlInstance', 'Type': 'uint8_t', 'Range': 'PinControlInstance'}, {'Name': 'Address', 'Type': 'uint32_t'}, {'Name': 'Length', 'Type': 'uint32_t'}]
    _RSP_PARAMS = [{'Name': 'CsPinControlInstance', 'Type': 'uint8_t'}, {'Name': 'Address', 'Type': 'uint32_t'}, {'Name': 'Length', 'Type': 'uint32_t'}]

    @classmethod
    def builder(cls, cs_pin_control_instance, address, length):
        msg = cls()
        msg.parameters = {
            "CsPinControlInstance": cs_pin_control_instance,
            "Address": address,
            "Length": length,
        }
        return msg


class MsgSpiFlashRead(MessageDefinition):
    _MSG_NAME = "SpiFlashRead"
    _CMD_CODE = 18
    _CMD_PARAMS = [{'Name': 'CsPinControlInstance', 'Type': 'uint8_t', 'Range': 'PinControlInstance'}, {'Name': 'Address', 'Type': 'uint32_t'}, {'Name': 'Length', 'Type': 'uint32_t', 'Range': 'BufferLimited'}]
    _RSP_PARAMS = [{'Name': 'CsPinControlInstance', 'Type': 'uint8_t'}, {'Name': 'Address', 'Type': 'uint32_t'}, {'Name': 'Data', 'Type': 'uint8_t *'}]

    @classmethod
    def builder(cls, cs_pin_control_instance, address, length):
        msg = cls()
        msg.parameters = {
            "CsPinControlInstance": cs_pin_control_instance,
            "Address": address,
            "Length": length,
        }
        return msg


class MsgSpiFlashWrite(MessageDefinition):
    _MSG_NAME = "SpiFlashWrite"
    _CMD_CODE = 19
    _CMD_PARAMS = [{'Name': 'CsPinControlInstance', 'Type': 'uint8_t', 'Range': 'PinControlInstance'}, {'Name': 'Address', 'Type': 'uint32_t'}, {'Name': 'Data', 'Type': 'uint8_t *'}]
    _RSP_PARAMS = [{'Name': 'CsPinControlInstance', 'Type': 'uint8_t'}, {'Name': 'Address', 'Type': 'uint32_t'}, {'Name': 'Length', 'Type': 'uint32_t'}]

    @classmethod
    def builder(cls, cs_pin_control_instance, address, data):
        msg = cls()
        msg.parameters = {
            "CsPinControlInstance": cs_pin_control_instance,
            "Address": address,
            "Data": data,
        }
        return msg


class MsgAccelInit(MessageDefinition):
    _MSG_NAME = "AccelInit"
    _CMD_CODE = 20
    _CMD_PARAMS = [{'Name': 'Modify', 'Type': 'uint8_t', 'Enum': 'Modify', 'Default': 0}, {'Name': 'ChipSelectPin', 'Type': 'int8_t', 'Description': 'Set to 0xFF for I2C mode.', 'Range': 'OptionalPinIndex', 'Default': -1}, {'Name': 'Int1Pin', 'Type': 'int8_t', 'Range': 'OptionalPinIndex', 'Default': -1}, {'Name': 'Int2Pin', 'Type': 'int8_t', 'Range': 'OptionalPinIndex', 'Default': -1}]
    _RSP_PARAMS = [{'Name': 'AccelModel', 'Type': 'uint8_t', 'Enum': 'SensorModel'}, {'Name': 'CsPinControlInstance', 'Type': 'uint8_t'}, {'Name': 'Int1Pin', 'Type': 'uint8_t'}, {'Name': 'Int2Pin', 'Type': 'uint8_t'}]

    @classmethod
    def builder(cls, modify=0, chip_select_pin=-1, int1_pin=-1, int2_pin=-1):
        msg = cls()
        msg.parameters = {
            "Modify": modify,
            "ChipSelectPin": chip_select_pin,
            "Int1Pin": int1_pin,
            "Int2Pin": int2_pin,
        }
        return msg


class MsgAccelUninit(MessageDefinition):
    _MSG_NAME = "AccelUninit"
    _CMD_CODE = 21
    _CMD_PARAMS = []
    _RSP_PARAMS = []

    @classmethod
    def builder(cls):
        msg = cls()
        msg.parameters = {
        }
        return msg


class MsgAccelStream(MessageDefinition):
    _MSG_NAME = "AccelStream"
    _CMD_CODE = 22
    _CMD_PARAMS = [{'Name': 'DataRange', 'Type': 'uint8_t', 'Enum': 'AccelDataRange'}, {'Name': 'DataRate', 'Type': 'uint8_t', 'Enum': 'AccelDataRate'}, {'Name': 'Watermark', 'Type': 'uint8_t', 'Range': 'AccelWatermark'}]
    _RSP_PARAMS = [{'Name': 'DataRange', 'Type': 'uint8_t', 'Enum': 'AccelDataRange'}, {'Name': 'DataRate', 'Type': 'uint8_t', 'Enum': 'AccelDataRate'}, {'Name': 'Watermark', 'Type': 'uint8_t'}, {'Name': 'AccelData', 'Type': 'uint8_t *'}]

    @classmethod
    def builder(cls, data_range, data_rate, watermark):
        msg = cls()
        msg.parameters = {
            "DataRange": data_range,
            "DataRate": data_rate,
            "Watermark": watermark,
        }
        return msg


class MsgSensorEvent(MessageDefinition):
    _MSG_NAME = "SensorEvent"
    _CMD_CODE = 23
    _CMD_PARAMS = []
    _RSP_PARAMS = [{'Name': 'EventType', 'Type': 'uint8_t', 'Enum': 'SensorEvent'}]

    @classmethod
    def builder(cls):
        msg = cls()
        msg.parameters = {
        }
        return msg


class MsgGyroInit(MessageDefinition):
    _MSG_NAME = "GyroInit"
    _CMD_CODE = 24
    _CMD_PARAMS = [{'Name': 'Modify', 'Type': 'uint8_t', 'Enum': 'Modify', 'Default': 0}, {'Name': 'GyroModel', 'Type': 'uint8_t', 'Enum': 'SensorModel', 'Default': 2}, {'Name': 'ChipSelectPin', 'Type': 'int8_t', 'Range': 'OptionalPinIndex', 'Description': 'Set to 0xFF for I2C mode.', 'Default': -1}, {'Name': 'Int1Pin', 'Type': 'int8_t', 'Range': 'OptionalPinIndex', 'Default': -1}, {'Name': 'Int2Pin', 'Type': 'int8_t', 'Range': 'OptionalPinIndex', 'Default': -1}]
    _RSP_PARAMS = [{'Name': 'GyroModel', 'Type': 'uint8_t', 'Enum': 'SensorModel'}, {'Name': 'CsPinControlInstance', 'Type': 'uint8_t'}, {'Name': 'Int1Pin', 'Type': 'uint8_t'}, {'Name': 'Int2Pin', 'Type': 'uint8_t'}]

    @classmethod
    def builder(cls, modify=0, gyro_model=2, chip_select_pin=-1, int1_pin=-1, int2_pin=-1):
        msg = cls()
        msg.parameters = {
            "Modify": modify,
            "GyroModel": gyro_model,
            "ChipSelectPin": chip_select_pin,
            "Int1Pin": int1_pin,
            "Int2Pin": int2_pin,
        }
        return msg


class MsgGyroUninit(MessageDefinition):
    _MSG_NAME = "GyroUninit"
    _CMD_CODE = 25
    _CMD_PARAMS = []
    _RSP_PARAMS = []

    @classmethod
    def builder(cls):
        msg = cls()
        msg.parameters = {
        }
        return msg


class MsgGyroStream(MessageDefinition):
    _MSG_NAME = "GyroStream"
    _CMD_CODE = 26
    _CMD_PARAMS = [{'Name': 'DataRange', 'Type': 'uint8_t', 'Enum': 'GyroDataRange'}, {'Name': 'DataRate', 'Type': 'uint8_t', 'Enum': 'GyroDataRate'}, {'Name': 'Watermark', 'Type': 'uint8_t', 'Range': 'GyroWatermark'}]
    _RSP_PARAMS = [{'Name': 'DataRange', 'Type': 'uint8_t', 'Enum': 'GyroDataRange'}, {'Name': 'DataRate', 'Type': 'uint8_t', 'Enum': 'GyroDataRate'}, {'Name': 'Watermark', 'Type': 'uint8_t'}, {'Name': 'GyroData', 'Type': 'uint8_t *'}]

    @classmethod
    def builder(cls, data_range, data_rate, watermark):
        msg = cls()
        msg.parameters = {
            "DataRange": data_range,
            "DataRate": data_rate,
            "Watermark": watermark,
        }
        return msg


class MsgPinAnalogInput(MessageDefinition):
    _MSG_NAME = "PinAnalogInput"
    _CMD_CODE = 27
    _CMD_PARAMS = [{'Name': 'Instance', 'Type': 'uint8_t', 'Range': 'AnalogInputInstance', 'Index': True}, {'Name': 'Modify', 'Type': 'uint8_t', 'Enum': 'Modify', 'Default': 0}, {'Name': 'PullConfig', 'Type': 'uint8_t', 'Enum': 'SaadcPullConfig', 'Default': 0}, {'Name': 'Gain', 'Type': 'uint8_t', 'Enum': 'SaadcGain', 'Default': 0}, {'Name': 'RefSel', 'Type': 'uint8_t', 'Enum': 'SaadcRefSel', 'Default': 0}, {'Name': 'Tacq', 'Type': 'uint8_t', 'Enum': 'SaadcTacq'}]
    _RSP_PARAMS = [{'Name': 'Instance', 'Type': 'uint8_t', 'Index': True}, {'Name': 'PullConfig', 'Type': 'uint8_t', 'Enum': 'SaadcPullConfig'}, {'Name': 'Gain', 'Type': 'uint8_t', 'Enum': 'SaadcGain'}, {'Name': 'RefSel', 'Type': 'uint8_t', 'Enum': 'SaadcRefSel'}, {'Name': 'Tacq', 'Type': 'uint8_t', 'Enum': 'SaadcTacq'}, {'Name': 'Measurement', 'Type': 'int16_t'}]

    @classmethod
    def builder(cls, instance, tacq, modify=0, pull_config=0, gain=0, ref_sel=0):
        msg = cls()
        msg.parameters = {
            "Instance": instance,
            "Modify": modify,
            "PullConfig": pull_config,
            "Gain": gain,
            "RefSel": ref_sel,
            "Tacq": tacq,
        }
        return msg


class MsgAnalogStream(MessageDefinition):
    _MSG_NAME = "AnalogStream"
    _CMD_CODE = 28
    _CMD_PARAMS = [{'Name': 'Instance', 'Type': 'uint8_t', 'Range': 'AnalogInputInstance', 'Index': True}, {'Name': 'SamplePeriodMicroSec', 'Type': 'uint32_t', 'Range': 'AdcSamplePeriod'}, {'Name': 'Watermark', 'Type': 'uint8_t', 'Range': 'AdcWatermark'}, {'Name': 'Gain', 'Type': 'uint8_t', 'Enum': 'SaadcGain'}, {'Name': 'RefSel', 'Type': 'uint8_t', 'Enum': 'SaadcRefSel'}]
    _RSP_PARAMS = [{'Name': 'Instance', 'Type': 'uint8_t', 'Index': True}, {'Name': 'SamplePeriodMicroSec', 'Type': 'uint32_t'}, {'Name': 'Watermark', 'Type': 'uint8_t'}, {'Name': 'Gain', 'Type': 'uint8_t', 'Enum': 'SaadcGain'}, {'Name': 'RefSel', 'Type': 'uint8_t', 'Enum': 'SaadcRefSel'}, {'Name': 'AdcData', 'Type': 'uint8_t *'}]

    @classmethod
    def builder(cls, instance, sample_period_micro_sec, watermark, gain, ref_sel):
        msg = cls()
        msg.parameters = {
            "Instance": instance,
            "SamplePeriodMicroSec": sample_period_micro_sec,
            "Watermark": watermark,
            "Gain": gain,
            "RefSel": ref_sel,
        }
        return msg


class MsgPinLedData(MessageDefinition):
    _MSG_NAME = "PinLedData"
    _CMD_CODE = 29
    _CMD_PARAMS = [{'Name': 'Modify', 'Type': 'uint8_t', 'Enum': 'Modify', 'Default': 0}, {'Name': 'RgbColorBytes', 'Type': 'uint8_t', 'Enum': 'RgbColorBytes', 'Default': 1}, {'Name': 'LedLength', 'Type': 'uint16_t', 'Range': 'LedLength'}, {'Name': 'AnimationLength', 'Type': 'uint16_t', 'Default': 0}, {'Name': 'AnimationPeriodMs', 'Type': 'uint16_t', 'Range': 'AnimationPeriod'}, {'Name': 'AnimationType', 'Type': 'uint8_t', 'Enum': 'AnimationType'}, {'Name': 'Encoding', 'Type': 'uint8_t *', 'DataStructure': 'RgbRunLengthEncoding', 'MaxLength': 'BufferLimited'}]
    _RSP_PARAMS = [{'Name': 'RgbColorBytes', 'Type': 'uint8_t', 'Enum': 'RgbColorBytes'}, {'Name': 'LedLength', 'Type': 'uint16_t'}, {'Name': 'AnimationLength', 'Type': 'uint16_t'}, {'Name': 'AnimationPeriodMs', 'Type': 'uint32_t'}, {'Name': 'AnimationType', 'Type': 'uint8_t'}, {'Name': 'Encoding', 'Type': 'uint8_t *', 'DataStructure': 'RgbRunLengthEncoding'}]

    @classmethod
    def builder(cls, led_length, animation_period_ms, animation_type, encoding, modify=0, rgb_color_bytes=1, animation_length=0):
        msg = cls()
        msg.parameters = {
            "Modify": modify,
            "RgbColorBytes": rgb_color_bytes,
            "LedLength": led_length,
            "AnimationLength": animation_length,
            "AnimationPeriodMs": animation_period_ms,
            "AnimationType": animation_type,
            "Encoding": encoding,
        }
        return msg


class MsgPinServoPulse(MessageDefinition):
    _MSG_NAME = "PinServoPulse"
    _CMD_CODE = 30
    _CMD_PARAMS = [{'Name': 'Instance', 'Type': 'uint8_t', 'Range': 'ServoInstance', 'Index': True}, {'Name': 'Channel', 'Type': 'uint8_t', 'Range': 'ServoChannel'}, {'Name': 'Modify', 'Type': 'uint8_t', 'Enum': 'Modify', 'Default': 0}, {'Name': 'PwmLowLimit', 'Type': 'uint16_t', 'Range': 'ServoPwm', 'Default': 600}, {'Name': 'PwmHighLimit', 'Type': 'uint16_t', 'Range': 'ServoPwm', 'Default': 2500}, {'Name': 'DownSpeedLimit', 'Type': 'uint16_t', 'Range': 'ServoPwm', 'Default': 1900}, {'Name': 'UpSpeedLimit', 'Type': 'uint16_t', 'Range': 'ServoPwm', 'Default': 1900}, {'Name': 'ServoName', 'Type': 'uint8_t *', 'MaxLength': 24}]
    _RSP_PARAMS = [{'Name': 'Instance', 'Type': 'uint8_t', 'Index': True}, {'Name': 'Channel', 'Type': 'uint8_t'}, {'Name': 'PwmLowLimit', 'Type': 'uint16_t'}, {'Name': 'PwmHighLimit', 'Type': 'uint16_t'}, {'Name': 'DownSpeedLimit', 'Type': 'uint16_t'}, {'Name': 'UpSpeedLimit', 'Type': 'uint16_t'}, {'Name': 'ServoName', 'Type': 'uint8_t *'}]

    @classmethod
    def builder(cls, instance, channel, servo_name, modify=0, pwm_low_limit=600, pwm_high_limit=2500, down_speed_limit=1900, up_speed_limit=1900):
        msg = cls()
        msg.parameters = {
            "Instance": instance,
            "Channel": channel,
            "Modify": modify,
            "PwmLowLimit": pwm_low_limit,
            "PwmHighLimit": pwm_high_limit,
            "DownSpeedLimit": down_speed_limit,
            "UpSpeedLimit": up_speed_limit,
            "ServoName": servo_name,
        }
        return msg


class MsgServoControl(MessageDefinition):
    _MSG_NAME = "ServoControl"
    _CMD_CODE = 31
    _CMD_PARAMS = [{'Name': 'Instance', 'Type': 'uint8_t', 'Range': 'ServoInstance', 'Index': True}, {'Name': 'Channel', 'Type': 'uint8_t', 'Range': 'ServoChannel'}, {'Name': 'Modify', 'Type': 'uint8_t', 'Enum': 'Modify', 'Default': 0}, {'Name': 'PulseWidth', 'Type': 'uint16_t', 'Range': 'ServoPwm'}, {'Name': 'MoveSpeed', 'Type': 'uint16_t', 'Range': 'ServoPwm', 'Default': 1900, 'Description': 'Maximum change per 20ms from the current pulse width.'}]
    _RSP_PARAMS = [{'Name': 'Instance', 'Type': 'uint8_t', 'Index': True}, {'Name': 'Channel', 'Type': 'uint8_t'}, {'Name': 'PulseWidth', 'Type': 'uint16_t'}, {'Name': 'MoveSpeed', 'Type': 'uint16_t'}]

    @classmethod
    def builder(cls, instance, channel, pulse_width, modify=0, move_speed=1900):
        msg = cls()
        msg.parameters = {
            "Instance": instance,
            "Channel": channel,
            "Modify": modify,
            "PulseWidth": pulse_width,
            "MoveSpeed": move_speed,
        }
        return msg


class MsgPinRobotis(MessageDefinition):
    _MSG_NAME = "PinRobotis"
    _CMD_CODE = 32
    _CMD_PARAMS = [{'Name': 'Modify', 'Type': 'uint8_t', 'Enum': 'Modify', 'Default': 0}, {'Name': 'ServoId', 'Type': 'uint8_t', 'Index': True}, {'Name': 'ServoName', 'Type': 'uint8_t *'}]
    _RSP_PARAMS = [{'Name': 'ServoId', 'Type': 'uint8_t', 'Index': True}, {'Name': 'ServoName', 'Type': 'uint8_t *'}]

    @classmethod
    def builder(cls, servo_id, servo_name, modify=0):
        msg = cls()
        msg.parameters = {
            "Modify": modify,
            "ServoId": servo_id,
            "ServoName": servo_name,
        }
        return msg


class MsgRobotisCommand(MessageDefinition):
    _MSG_NAME = "RobotisCommand"
    _CMD_CODE = 33
    _CMD_PARAMS = [{'Name': 'ServoId', 'Type': 'uint8_t', 'Index': True}, {'Name': 'Instruction', 'Type': 'uint8_t', 'Description': '1:Ping, 2:Read, 3:Write, 4:RegWrite, 5:Action, 6:FactoryReset, 7:Reboot, 8:SyncWrite, 9:BulkRead'}, {'Name': 'Parameters', 'Type': 'uint8_t *'}]
    _RSP_PARAMS = [{'Name': 'BytesSent', 'Type': 'uint32_t'}, {'Name': 'ResponseServoId', 'Type': 'uint8_t', 'Index': True}, {'Name': 'ResponseErrorStatus', 'Type': 'uint8_t'}, {'Name': 'ResponseParameters', 'Type': 'uint8_t *'}]

    @classmethod
    def builder(cls, servo_id, instruction, parameters):
        msg = cls()
        msg.parameters = {
            "ServoId": servo_id,
            "Instruction": instruction,
            "Parameters": parameters,
        }
        return msg


class MsgPinPwmChannel(MessageDefinition):
    _MSG_NAME = "PinPwmChannel"
    _CMD_CODE = 34
    _CMD_PARAMS = [{'Name': 'Instance', 'Type': 'uint8_t', 'Range': 'PwmInstance', 'Index': True}, {'Name': 'Modify', 'Type': 'uint8_t', 'Enum': 'Modify'}, {'Name': 'MinPeriod', 'Type': 'uint16_t'}, {'Name': 'MinPulseWidth', 'Type': 'uint16_t'}, {'Name': 'MaxPulseWidth', 'Type': 'uint16_t'}, {'Name': 'ChannelName', 'Type': 'uint8_t *'}]
    _RSP_PARAMS = [{'Name': 'Instance', 'Type': 'uint8_t', 'Index': True}, {'Name': 'MinPeriod', 'Type': 'uint16_t'}, {'Name': 'MinPulseWidth', 'Type': 'uint16_t'}, {'Name': 'MaxPulseWidth', 'Type': 'uint16_t'}, {'Name': 'ChannelName', 'Type': 'uint8_t *'}]

    @classmethod
    def builder(cls, instance, modify, min_period, min_pulse_width, max_pulse_width, channel_name):
        msg = cls()
        msg.parameters = {
            "Instance": instance,
            "Modify": modify,
            "MinPeriod": min_period,
            "MinPulseWidth": min_pulse_width,
            "MaxPulseWidth": max_pulse_width,
            "ChannelName": channel_name,
        }
        return msg


class MsgPwmChannelControl(MessageDefinition):
    _MSG_NAME = "PwmChannelControl"
    _CMD_CODE = 35
    _CMD_PARAMS = [{'Name': 'Instance', 'Type': 'uint8_t', 'Range': 'PwmInstance', 'Index': True}, {'Name': 'Modify', 'Type': 'uint8_t', 'Enum': 'Modify'}, {'Name': 'PulseWidth', 'Type': 'uint16_t'}]
    _RSP_PARAMS = [{'Name': 'Instance', 'Type': 'uint8_t', 'Index': True}, {'Name': 'Channel', 'Type': 'uint8_t'}, {'Name': 'PulseWidth', 'Type': 'uint16_t'}]

    @classmethod
    def builder(cls, instance, modify, pulse_width):
        msg = cls()
        msg.parameters = {
            "Instance": instance,
            "Modify": modify,
            "PulseWidth": pulse_width,
        }
        return msg


class MsgAdvertisingConfig(MessageDefinition):
    _MSG_NAME = "AdvertisingConfig"
    _CMD_CODE = 36
    _CMD_PARAMS = [{'Name': 'Modify', 'Type': 'uint8_t', 'Enum': 'Modify', 'Default': 0}, {'Name': 'Interval_0p625ms', 'Type': 'uint16_t', 'Range': 'AdvIntervalRange', 'Default': 300}, {'Name': 'TxPower', 'Type': 'uint8_t', 'Enum': 'TxPower', 'Default': 9}, {'Name': 'PrimaryPhyCoding', 'Type': 'uint8_t', 'Enum': 'PhyCoding', 'Default': 0}, {'Name': 'SecondaryPhyCoding', 'Type': 'uint8_t', 'Enum': 'PhyCoding', 'Default': 0}, {'Name': 'AdvType', 'Type': 'uint8_t', 'Enum': 'AdvertisingType', 'Default': 0}, {'Name': 'AdvName', 'Type': 'uint8_t', 'Enum': 'AdvertiseIn', 'Default': 1}, {'Name': 'AdvService128Bit', 'Type': 'uint8_t', 'Enum': 'AdvertiseIn', 'Default': 2}, {'Name': 'AdvMfrSpecificData', 'Type': 'uint8_t', 'Enum': 'AdvertiseIn', 'Default': 2}, {'Name': 'MfrSpecificDataLength', 'Type': 'uint8_t', 'Default': 16}]
    _RSP_PARAMS = [{'Name': 'Interval_0p625ms', 'Type': 'uint16_t', 'Range': 'AdvIntervalRange'}, {'Name': 'TxPower', 'Type': 'uint8_t'}, {'Name': 'PrimaryPhyCoding', 'Type': 'uint8_t', 'Enum': 'PhyCoding'}, {'Name': 'SecondaryPhyCoding', 'Type': 'uint8_t', 'Enum': 'PhyCoding'}, {'Name': 'AdvType', 'Type': 'uint8_t', 'Enum': 'AdvertisingType', 'Default': 0}, {'Name': 'AdvName', 'Type': 'uint8_t', 'Enum': 'AdvertiseIn'}, {'Name': 'AdvService128Bit', 'Type': 'uint8_t', 'Enum': 'AdvertiseIn'}, {'Name': 'AdvMfrSpecificData', 'Type': 'uint8_t', 'Enum': 'AdvertiseIn'}, {'Name': 'MfrSpecificDataLength', 'Type': 'uint8_t'}]

    @classmethod
    def builder(cls, modify=0, interval_0p625ms=300, tx_power=9, primary_phy_coding=0, secondary_phy_coding=0, adv_type=0, adv_name=1, adv_service128_bit=2, adv_mfr_specific_data=2, mfr_specific_data_length=16):
        msg = cls()
        msg.parameters = {
            "Modify": modify,
            "Interval_0p625ms": interval_0p625ms,
            "TxPower": tx_power,
            "PrimaryPhyCoding": primary_phy_coding,
            "SecondaryPhyCoding": secondary_phy_coding,
            "AdvType": adv_type,
            "AdvName": adv_name,
            "AdvService128Bit": adv_service128_bit,
            "AdvMfrSpecificData": adv_mfr_specific_data,
            "MfrSpecificDataLength": mfr_specific_data_length,
        }
        return msg


class MsgConnParamUpdate(MessageDefinition):
    _MSG_NAME = "ConnParamUpdate"
    _CMD_CODE = 37
    _CMD_PARAMS = [{'Name': 'Modify', 'Type': 'uint8_t', 'Enum': 'Modify', 'Default': 0}, {'Name': 'ConnHandle', 'Type': 'uint8_t', 'Default': 0, 'Index': True}, {'Name': 'ConnIntervalMin_1p25ms', 'Type': 'uint16_t', 'Range': 'ConnInterval', 'Default': 6}, {'Name': 'ConnIntervalMax_1p25ms', 'Type': 'uint16_t', 'Range': 'ConnInterval', 'Default': 16}, {'Name': 'SlaveLatency', 'Type': 'uint8_t', 'Default': 4}, {'Name': 'ConnSupTimeout_10ms', 'Type': 'uint16_t', 'Default': 400}]
    _RSP_PARAMS = [{'Name': 'ConnHandle', 'Type': 'uint8_t', 'Index': True}, {'Name': 'ConnIntervalMin_1p25ms', 'Type': 'uint16_t'}, {'Name': 'ConnIntervalMax_1p25ms', 'Type': 'uint16_t'}, {'Name': 'SlaveLatency', 'Type': 'uint8_t'}, {'Name': 'ConnSupTimeout_10ms', 'Type': 'uint16_t'}, {'Name': 'ConnectionStatus', 'Type': 'uint8_t', 'Enum': 'ConnectionStatus'}, {'Name': 'PeerAddress', 'Type': 'uint8_t *'}]

    @classmethod
    def builder(cls, modify=0, conn_handle=0, conn_interval_min_1p25ms=6, conn_interval_max_1p25ms=16, slave_latency=4, conn_sup_timeout_10ms=400):
        msg = cls()
        msg.parameters = {
            "Modify": modify,
            "ConnHandle": conn_handle,
            "ConnIntervalMin_1p25ms": conn_interval_min_1p25ms,
            "ConnIntervalMax_1p25ms": conn_interval_max_1p25ms,
            "SlaveLatency": slave_latency,
            "ConnSupTimeout_10ms": conn_sup_timeout_10ms,
        }
        return msg


class MsgRtcSync(MessageDefinition):
    _MSG_NAME = "RtcSync"
    _CMD_CODE = 38
    _CMD_PARAMS = [{'Name': 'Modify', 'Type': 'uint8_t', 'Enum': 'Modify'}, {'Name': 'SyncValue', 'Type': 'uint32_t'}]
    _RSP_PARAMS = [{'Name': 'CurrentClock', 'Type': 'uint32_t'}, {'Name': 'SyncValue', 'Type': 'uint32_t'}]

    @classmethod
    def builder(cls, modify, sync_value):
        msg = cls()
        msg.parameters = {
            "Modify": modify,
            "SyncValue": sync_value,
        }
        return msg


class MsgRssiStream(MessageDefinition):
    _MSG_NAME = "RssiStream"
    _CMD_CODE = 39
    _CMD_PARAMS = [{'Name': 'ConnectionHandle', 'Type': 'uint8_t', 'Default': 0, 'Index': True}, {'Name': 'SamplePeriod_100ms', 'Type': 'uint16_t', 'Range': 'RssiSamplePeriod', 'Default': 10}, {'Name': 'Watermark', 'Type': 'uint8_t', 'Range': 'RssiWatermark'}]
    _RSP_PARAMS = [{'Name': 'ConnectionHandle', 'Type': 'uint8_t', 'Index': True}, {'Name': 'SamplePeriod_100ms', 'Type': 'uint16_t'}, {'Name': 'Watermark', 'Type': 'uint8_t'}, {'Name': 'RssiData', 'Type': 'uint8_t *'}]

    @classmethod
    def builder(cls, watermark, connection_handle=0, sample_period_100ms=10):
        msg = cls()
        msg.parameters = {
            "ConnectionHandle": connection_handle,
            "SamplePeriod_100ms": sample_period_100ms,
            "Watermark": watermark,
        }
        return msg


class MsgAccelSteps(MessageDefinition):
    _MSG_NAME = "AccelSteps"
    _CMD_CODE = 40
    _CMD_PARAMS = [{'Name': 'ResetCount', 'Type': 'uint8_t', 'Range': 'Boolean', 'Default': 1}, {'Name': 'SamplePeriod_100ms', 'Type': 'uint16_t', 'Range': 'RssiSamplePeriod', 'Default': 600}]
    _RSP_PARAMS = [{'Name': 'SamplePeriod_100ms', 'Type': 'uint16_t'}, {'Name': 'CurrentStepCount', 'Type': 'uint32_t'}]

    @classmethod
    def builder(cls, reset_count=1, sample_period_100ms=600):
        msg = cls()
        msg.parameters = {
            "ResetCount": reset_count,
            "SamplePeriod_100ms": sample_period_100ms,
        }
        return msg


class MsgLinkParamUpdate(MessageDefinition):
    _MSG_NAME = "LinkParamUpdate"
    _CMD_CODE = 41
    _CMD_PARAMS = [{'Name': 'Modify', 'Type': 'uint8_t', 'Enum': 'Modify', 'Default': 0}, {'Name': 'ConnHandle', 'Type': 'uint8_t', 'Default': 0, 'Index': True}, {'Name': 'ConnTxPower', 'Type': 'uint8_t', 'Enum': 'TxPower', 'Default': 9}, {'Name': 'PhyCoding', 'Type': 'uint8_t', 'Enum': 'PhyCoding', 'Default': 0}, {'Name': 'BytesPerPacket', 'Type': 'uint16_t', 'Range': 'BytesPerPacket', 'Default': 27}, {'Name': 'PacketsPerInterval', 'Type': 'uint16_t', 'Range': 'PacketsPerInterval', 'Default': 4}]
    _RSP_PARAMS = [{'Name': 'ConnTxPower', 'Type': 'uint8_t'}, {'Name': 'ConnHandle', 'Type': 'uint8_t', 'Index': True}, {'Name': 'PhyCoding', 'Type': 'uint8_t', 'Enum': 'PhyCoding'}, {'Name': 'BytesPerPacket', 'Type': 'uint16_t'}, {'Name': 'PacketsPerInterval', 'Type': 'uint16_t'}]

    @classmethod
    def builder(cls, modify=0, conn_handle=0, conn_tx_power=9, phy_coding=0, bytes_per_packet=27, packets_per_interval=4):
        msg = cls()
        msg.parameters = {
            "Modify": modify,
            "ConnHandle": conn_handle,
            "ConnTxPower": conn_tx_power,
            "PhyCoding": phy_coding,
            "BytesPerPacket": bytes_per_packet,
            "PacketsPerInterval": packets_per_interval,
        }
        return msg


class MsgAccelTapSensitivity(MessageDefinition):
    _MSG_NAME = "AccelTapSensitivity"
    _CMD_CODE = 42
    _CMD_PARAMS = [{'Name': 'Sensitivity', 'Type': 'uint8_t', 'Range': 'TapSensitivity'}]
    _RSP_PARAMS = [{'Name': 'Sensitivity', 'Type': 'uint8_t'}]

    @classmethod
    def builder(cls, sensitivity):
        msg = cls()
        msg.parameters = {
            "Sensitivity": sensitivity,
        }
        return msg


class MsgCentralControl(MessageDefinition):
    _MSG_NAME = "CentralControl"
    _CMD_CODE = 43
    _CMD_PARAMS = [{'Name': 'ScanEnable', 'Type': 'uint8_t', 'Range': 'Boolean', 'Default': 1}, {'Name': 'Connect', 'Type': 'uint8_t', 'Range': 'Boolean', 'Default': 0}, {'Name': 'ActiveScan', 'Type': 'uint8_t', 'Range': 'Boolean', 'Default': 0}, {'Name': 'Interval_0p625us', 'Type': 'uint16_t', 'Range': 'ScanLimits', 'Default': 96}, {'Name': 'Window_0p625us', 'Type': 'uint16_t', 'Range': 'ScanLimits', 'Default': 48}, {'Name': 'Timeout10ms', 'Type': 'uint16_t', 'Range': 'ScanTimeout', 'Default': 1000}, {'Name': 'FilterType', 'Type': 'uint8_t', 'Enum': 'FilterType', 'Default': 2}, {'Name': 'FilterValue', 'Type': 'uint8_t *', 'Default': 'Bluemo'}]
    _RSP_PARAMS = [{'Name': 'ScanEnable', 'Type': 'uint8_t', 'Range': 'Boolean'}, {'Name': 'Connect', 'Type': 'uint8_t', 'Range': 'Boolean'}, {'Name': 'ActiveScan', 'Type': 'uint8_t', 'Range': 'Boolean'}, {'Name': 'Interval_0p625us', 'Type': 'uint16_t', 'Range': 'ScanLimits'}, {'Name': 'Window_0p625us', 'Type': 'uint16_t', 'Range': 'ScanLimits'}, {'Name': 'Timeout10ms', 'Type': 'uint16_t', 'Range': 'ScanTimeout'}, {'Name': 'FilterType', 'Type': 'uint8_t', 'Enum': 'FilterType'}, {'Name': 'FilterValue', 'Type': 'uint8_t *', 'MaxLength': 32}]

    @classmethod
    def builder(cls, scan_enable=1, connect=0, active_scan=0, interval_0p625us=96, window_0p625us=48, timeout10ms=1000, filter_type=2, filter_value="Bluemo"):
        msg = cls()
        msg.parameters = {
            "ScanEnable": scan_enable,
            "Connect": connect,
            "ActiveScan": active_scan,
            "Interval_0p625us": interval_0p625us,
            "Window_0p625us": window_0p625us,
            "Timeout10ms": timeout10ms,
            "FilterType": filter_type,
            "FilterValue": filter_value,
        }
        return msg


class MsgScanResult(MessageDefinition):
    _MSG_NAME = "ScanResult"
    _CMD_CODE = 44
    _CMD_PARAMS = []
    _RSP_PARAMS = [{'Name': 'ReportType', 'Type': 'uint16_t'}, {'Name': 'PrimaryPhy', 'Type': 'uint8_t'}, {'Name': 'SecondaryPhy', 'Type': 'uint8_t'}, {'Name': 'TxPower', 'Type': 'int8_t'}, {'Name': 'Rssi', 'Type': 'int8_t'}, {'Name': 'ChannelIndex', 'Type': 'uint8_t'}, {'Name': 'AdvData', 'Type': 'uint8_t *', 'DataStructure': 'ScanResult'}]

    @classmethod
    def builder(cls):
        msg = cls()
        msg.parameters = {
        }
        return msg


class MsgCentralAction(MessageDefinition):
    _MSG_NAME = "CentralAction"
    _CMD_CODE = 45
    _CMD_PARAMS = [{'Name': 'ConnHandle', 'Type': 'uint16_t', 'Index': True}, {'Name': 'ActionType', 'Type': 'uint8_t', 'Enum': 'ActionType', 'Default': 1}]
    _RSP_PARAMS = [{'Name': 'ConnHandle', 'Type': 'uint16_t', 'Index': True}, {'Name': 'ActionType', 'Type': 'uint8_t', 'Enum': 'ActionType'}, {'Name': 'VersionInfo', 'Type': 'uint8_t *'}]

    @classmethod
    def builder(cls, conn_handle, action_type=1):
        msg = cls()
        msg.parameters = {
            "ConnHandle": conn_handle,
            "ActionType": action_type,
        }
        return msg


class MsgRelayData(MessageDefinition):
    _MSG_NAME = "RelayData"
    _CMD_CODE = 46
    _CMD_PARAMS = [{'Name': 'ConnHandle', 'Type': 'uint16_t', 'Index': True}, {'Name': 'Payload', 'Type': 'uint8_t *'}]
    _RSP_PARAMS = [{'Name': 'ConnHandle', 'Type': 'uint16_t', 'Index': True}, {'Name': 'Payload', 'Type': 'uint8_t *'}]

    @classmethod
    def builder(cls, conn_handle, payload):
        msg = cls()
        msg.parameters = {
            "ConnHandle": conn_handle,
            "Payload": payload,
        }
        return msg


MSG_CLASS_BY_RSP_CODE = {
    128: MsgError,
    129: MsgDeviceName,
    130: MsgSoftReset,
    131: MsgMfrOtp,
    132: MsgLoopback,
    133: MsgPinConfig,
    134: MsgPinControl,
    135: MsgPinMonitor,
    136: MsgTwimInit,
    137: MsgTwimDisconnect,
    138: MsgTwimScan,
    139: MsgTwimTransfer,
    140: MsgSpiInit,
    141: MsgSpiDisconnect,
    142: MsgSpiTransfer,
    143: MsgSpiFlashInit,
    144: MsgSpiFlashDisconnect,
    145: MsgSpiFlashErase,
    146: MsgSpiFlashRead,
    147: MsgSpiFlashWrite,
    148: MsgAccelInit,
    149: MsgAccelUninit,
    150: MsgAccelStream,
    151: MsgSensorEvent,
    152: MsgGyroInit,
    153: MsgGyroUninit,
    154: MsgGyroStream,
    155: MsgPinAnalogInput,
    156: MsgAnalogStream,
    157: MsgPinLedData,
    158: MsgPinServoPulse,
    159: MsgServoControl,
    160: MsgPinRobotis,
    161: MsgRobotisCommand,
    162: MsgPinPwmChannel,
    163: MsgPwmChannelControl,
    164: MsgAdvertisingConfig,
    165: MsgConnParamUpdate,
    166: MsgRtcSync,
    167: MsgRssiStream,
    168: MsgAccelSteps,
    169: MsgLinkParamUpdate,
    170: MsgAccelTapSensitivity,
    171: MsgCentralControl,
    172: MsgScanResult,
    173: MsgCentralAction,
    174: MsgRelayData,
}
