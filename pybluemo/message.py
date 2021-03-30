import struct


class Consts(object):
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
            if param[Consts.MSG_PARAM_TYPE] == "uint8_t":
                format_str += "B"
            elif param[Consts.MSG_PARAM_TYPE] == "uint16_t":
                format_str += "H"
            elif param[Consts.MSG_PARAM_TYPE] == "int16_t":
                format_str += "h"
            elif param[Consts.MSG_PARAM_TYPE] == "uint32_t":
                format_str += "I"
            elif param[Consts.MSG_PARAM_TYPE] == "int32_t":
                format_str += "i"
            elif param[Consts.MSG_PARAM_NAME] == "float":
                format_str += "f"
            elif param[Consts.MSG_PARAM_TYPE] == "uint8_t *":
                format_str += "%ds" % len(self.get_param(param[Consts.MSG_PARAM_NAME]))
            if param[Consts.MSG_PARAM_NAME] in self.parameters:
                params_tuple += (self.get_param(param[Consts.MSG_PARAM_NAME]),)
            elif Consts.MSG_PARAM_DEFAULT in param:
                params_tuple += (param[Consts.MSG_PARAM_DEFAULT], )
            else:
                raise Exception("Required parameter not specified: %s" % param[Consts.MSG_PARAM_NAME])
        return struct.pack(format_str, *params_tuple)

    def _unpack(self, payload, param_list):
        format_str = "<"
        offset = 0
        for param in param_list:
            if param[Consts.MSG_PARAM_TYPE] == "uint8_t":
                format_str += "B"
                offset += 1
            elif param[Consts.MSG_PARAM_TYPE] == "uint16_t":
                format_str += "H"
                offset += 2
            elif param[Consts.MSG_PARAM_TYPE] == "int16_t":
                format_str += "h"
                offset += 2
            elif param[Consts.MSG_PARAM_TYPE] == "uint32_t":
                format_str += "I"
                offset += 4
            elif param[Consts.MSG_PARAM_TYPE] == "int32_t":
                format_str += "i"
                offset += 4
            elif param[Consts.MSG_PARAM_TYPE] == "float":
                format_str += "f"
                offset += 4
            elif param[Consts.MSG_PARAM_TYPE] == "uint8_t *":
                format_str += "%ds" % (len(payload) - offset)
        params_tuple = struct.unpack(format_str, payload)
        for i in range(len(param_list)):
            self.set_param(param_list[i][Consts.MSG_PARAM_NAME], params_tuple[i])

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
    CONTROL = 1
    MONITOR = 2
    ANALOG_INPUT = 3
    LED_DATA = 4
    TOTAL_FUNCTIONS = 5


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


class MsgDeviceConfig(MessageDefinition):
    _MSG_NAME = "DeviceConfig"
    _CMD_CODE = 1
    _CMD_PARAMS = [{'Name': 'Modify', 'Type': 'uint8_t', 'Enum': 'Modify', 'Default': 0}, {'Name': 'BatteryType', 'Type': 'uint8_t', 'Enum': 'BatteryType', 'Default': 0}, {'Name': 'HardwareRevision', 'Type': 'uint8_t', 'Default': 0}, {'Name': 'SerialNumber', 'Type': 'uint32_t', 'Default': 0}, {'Name': 'DeviceName', 'Type': 'uint8_t *'}]
    _RSP_PARAMS = [{'Name': 'BatteryType', 'Type': 'uint8_t', 'Enum': 'BATTERY_TYPE_E'}, {'Name': 'HardwareRevision', 'Type': 'uint8_t'}, {'Name': 'SerialNumber', 'Type': 'uint32_t'}, {'Name': 'DeviceName', 'Type': 'uint8_t *'}]

    @classmethod
    def builder(cls, device_name, modify=0, battery_type=0, hardware_revision=0, serial_number=0):
        msg = cls()
        msg.parameters = {
            "Modify": modify,
            "BatteryType": battery_type,
            "HardwareRevision": hardware_revision,
            "SerialNumber": serial_number,
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


class MsgLoopback(MessageDefinition):
    _MSG_NAME = "Loopback"
    _CMD_CODE = 3
    _CMD_PARAMS = [{'Name': 'Handle', 'Type': 'uint8_t'}, {'Name': 'Payload', 'Type': 'uint8_t *'}]
    _RSP_PARAMS = [{'Name': 'Handle', 'Type': 'uint8_t'}, {'Name': 'Payload', 'Type': 'uint8_t *'}]

    @classmethod
    def builder(cls, handle, payload):
        msg = cls()
        msg.parameters = {
            "Handle": handle,
            "Payload": payload,
        }
        return msg


class MsgPinConfig(MessageDefinition):
    _MSG_NAME = "PinConfig"
    _CMD_CODE = 4
    _CMD_PARAMS = [{'Name': 'Modify', 'Type': 'uint8_t', 'Enum': 'Modify', 'Default': 0}, {'Name': 'PinIndex', 'Type': 'uint8_t'}, {'Name': 'Function', 'Type': 'uint8_t', 'Enum': 'PinFunction', 'Default': 0}, {'Name': 'Instance', 'Type': 'uint8_t', 'Default': 255}]
    _RSP_PARAMS = [{'Name': 'PinIndex', 'Type': 'uint8_t'}, {'Name': 'Function', 'Type': 'uint8_t', 'Enum': 'PinFunction'}, {'Name': 'Instance', 'Type': 'uint8_t'}]

    @classmethod
    def builder(cls, pin_index, modify=0, function=0, instance=255):
        msg = cls()
        msg.parameters = {
            "Modify": modify,
            "PinIndex": pin_index,
            "Function": function,
            "Instance": instance,
        }
        return msg


class MsgPinControl(MessageDefinition):
    _MSG_NAME = "PinControl"
    _CMD_CODE = 5
    _CMD_PARAMS = [{'Name': 'Instance', 'Type': 'uint8_t'}, {'Name': 'Modify', 'Type': 'uint8_t', 'Enum': 'Modify', 'Default': 0}, {'Name': 'DriveStrength', 'Type': 'uint8_t', 'Enum': 'DriveStrength', 'Default': 0}, {'Name': 'OutputState', 'Type': 'uint8_t', 'Default': 0}, {'Name': 'PinControlName', 'Type': 'uint8_t *'}]
    _RSP_PARAMS = [{'Name': 'Instance', 'Type': 'uint8_t'}, {'Name': 'DriveConfig', 'Type': 'uint8_t'}, {'Name': 'OutputState', 'Type': 'uint8_t'}, {'Name': 'PinControlName', 'Type': 'uint8_t *'}]

    @classmethod
    def builder(cls, instance, pin_control_name, modify=0, drive_strength=0, output_state=0):
        msg = cls()
        msg.parameters = {
            "Instance": instance,
            "Modify": modify,
            "DriveStrength": drive_strength,
            "OutputState": output_state,
            "PinControlName": pin_control_name,
        }
        return msg


class MsgPinMonitor(MessageDefinition):
    _MSG_NAME = "PinMonitor"
    _CMD_CODE = 6
    _CMD_PARAMS = [{'Name': 'Instance', 'Type': 'uint8_t'}, {'Name': 'Modify', 'Type': 'uint8_t', 'Enum': 'MODIFY_E', 'Default': 0}, {'Name': 'PullConfig', 'Type': 'uint8_t', 'Enum': 'PULL_CFG_E', 'Default': 0}, {'Name': 'SenseConfig', 'Type': 'uint8_t', 'Enum': 'SENSE_CFG_E', 'Default': 0}, {'Name': 'PinMonitorName', 'Type': 'uint8_t *'}]
    _RSP_PARAMS = [{'Name': 'Instance', 'Type': 'uint8_t'}, {'Name': 'PullConfig', 'Type': 'uint8_t', 'Enum': 'PULL_CFG_E'}, {'Name': 'SenseConfig', 'Type': 'uint8_t', 'Enum': 'SENSE_CFG_E'}, {'Name': 'PinStatus', 'Type': 'uint8_t'}, {'Name': 'PinMonitorName', 'Type': 'uint8_t *'}]

    @classmethod
    def builder(cls, instance, pin_monitor_name, modify=0, pull_config=0, sense_config=0):
        msg = cls()
        msg.parameters = {
            "Instance": instance,
            "Modify": modify,
            "PullConfig": pull_config,
            "SenseConfig": sense_config,
            "PinMonitorName": pin_monitor_name,
        }
        return msg


class MsgTwimInit(MessageDefinition):
    _MSG_NAME = "TwimInit"
    _CMD_CODE = 7
    _CMD_PARAMS = [{'Name': 'Modify', 'Type': 'uint8_t', 'Enum': 'MODIFY_E', 'Default': 0}, {'Name': 'SclPin', 'Type': 'uint8_t', 'Default': 255}, {'Name': 'SdaPin', 'Type': 'uint8_t', 'Default': 255}, {'Name': 'Frequency', 'Type': 'uint8_t', 'Enum': 'I2C_FREQ_E', 'Default': 2}]
    _RSP_PARAMS = [{'Name': 'SclPin', 'Type': 'uint8_t'}, {'Name': 'SdaPin', 'Type': 'uint8_t'}, {'Name': 'Frequency', 'Type': 'uint8_t', 'Enum': 'I2C_FREQ_E'}]

    @classmethod
    def builder(cls, modify=0, scl_pin=255, sda_pin=255, frequency=2):
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
    _CMD_CODE = 8
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
    _CMD_CODE = 9
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
    _CMD_CODE = 10
    _CMD_PARAMS = [{'Name': 'Handle', 'Type': 'uint8_t', 'Description': 'Software specified operation handle copied by firmware to the response message'}, {'Name': 'SlaveAddress', 'Type': 'uint8_t'}, {'Name': 'ReadLength', 'Type': 'uint16_t'}, {'Name': 'WriteData', 'Type': 'uint8_t *'}]
    _RSP_PARAMS = [{'Name': 'Handle', 'Type': 'uint8_t', 'Description': 'Copy of Handle provided in command packet, not relevant to firmware, but this can be used to identify responses in software when multiple concurrent transfers are in progress.'}, {'Name': 'ActualWriteLength', 'Type': 'uint16_t', 'Description': 'Actual size written, if a write was attempted but no I2C ack is received, this will be zero.'}, {'Name': 'ActualReadData', 'Type': 'uint8_t *', 'Description': 'Actual size read, if a read was attempted but no I2C ack is received, this will be zero.'}]

    @classmethod
    def builder(cls, handle, slave_address, read_length, write_data):
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
    _CMD_CODE = 11
    _CMD_PARAMS = [{'Name': 'Modify', 'Type': 'uint8_t', 'Enum': 'MODIFY_E', 'Default': 0}, {'Name': 'ClkPin', 'Type': 'uint8_t', 'Default': 255}, {'Name': 'MosiPin', 'Type': 'uint8_t', 'Default': 255}, {'Name': 'MisoPin', 'Type': 'uint8_t', 'Default': 255}, {'Name': 'Frequency', 'Type': 'uint8_t', 'Enum': 'SpiFreq', 'Default': 3}]
    _RSP_PARAMS = [{'Name': 'ClkPin', 'Type': 'uint8_t'}, {'Name': 'MosiPin', 'Type': 'uint8_t'}, {'Name': 'MisoPin', 'Type': 'uint8_t'}, {'Name': 'Frequency', 'Type': 'uint8_t', 'Enum': 'SpiFreq'}]

    @classmethod
    def builder(cls, modify=0, clk_pin=255, mosi_pin=255, miso_pin=255, frequency=3):
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
    _CMD_CODE = 12
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
    _CMD_CODE = 13
    _CMD_PARAMS = [{'Name': 'Handle', 'Type': 'uint8_t'}, {'Name': 'CsPinControlInstance', 'Type': 'uint8_t'}, {'Name': 'ReadLength', 'Type': 'uint16_t'}, {'Name': 'WriteData', 'Type': 'uint8_t *'}]
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
    _CMD_CODE = 14
    _CMD_PARAMS = [{'Name': 'Modify', 'Type': 'uint8_t', 'Enum': 'Modify', 'Default': 0}, {'Name': 'ChipSelectPin', 'Type': 'uint8_t', 'Default': 255}, {'Name': 'WriteProtectPin', 'Type': 'uint8_t', 'Default': 255}, {'Name': 'ResetPin', 'Type': 'uint8_t', 'Default': 255}]
    _RSP_PARAMS = [{'Name': 'FlashModel', 'Type': 'uint8_t', 'Enum': 'FlashModel'}, {'Name': 'CsPinControlInstance', 'Type': 'uint8_t'}, {'Name': 'WpPinControlInstance', 'Type': 'uint8_t'}, {'Name': 'RstPinControlInstance', 'Type': 'uint8_t'}, {'Name': 'DeviceId', 'Type': 'uint8_t *'}]

    @classmethod
    def builder(cls, modify=0, chip_select_pin=255, write_protect_pin=255, reset_pin=255):
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
    _CMD_CODE = 15
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
    _CMD_CODE = 16
    _CMD_PARAMS = [{'Name': 'CsPinControlInstance', 'Type': 'uint8_t'}, {'Name': 'Address', 'Type': 'uint32_t'}, {'Name': 'Length', 'Type': 'uint32_t'}]
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
    _CMD_CODE = 17
    _CMD_PARAMS = [{'Name': 'CsPinControlInstance', 'Type': 'uint8_t'}, {'Name': 'Address', 'Type': 'uint32_t'}, {'Name': 'Length', 'Type': 'uint32_t'}]
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
    _CMD_CODE = 18
    _CMD_PARAMS = [{'Name': 'CsPinControlInstance', 'Type': 'uint8_t'}, {'Name': 'Address', 'Type': 'uint32_t'}, {'Name': 'Data', 'Type': 'uint8_t *'}]
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
    _CMD_CODE = 19
    _CMD_PARAMS = [{'Name': 'Modify', 'Type': 'uint8_t', 'Enum': 'Modify', 'Default': 0}, {'Name': 'AccelModel', 'Type': 'uint8_t', 'Enum': 'SensorModel'}, {'Name': 'ChipSelectPin', 'Type': 'uint8_t', 'Description': 'Set to 0xFF for I2C mode.', 'Default': 255}, {'Name': 'Int1Pin', 'Type': 'uint8_t', 'Default': 255}, {'Name': 'Int2Pin', 'Type': 'uint8_t', 'Default': 255}]
    _RSP_PARAMS = [{'Name': 'AccelModel', 'Type': 'uint8_t', 'Enum': 'SensorModel'}, {'Name': 'CsPinControlInstance', 'Type': 'uint8_t'}, {'Name': 'Int1Pin', 'Type': 'uint8_t'}, {'Name': 'Int2Pin', 'Type': 'uint8_t'}]

    @classmethod
    def builder(cls, accel_model, modify=0, chip_select_pin=255, int1_pin=255, int2_pin=255):
        msg = cls()
        msg.parameters = {
            "Modify": modify,
            "AccelModel": accel_model,
            "ChipSelectPin": chip_select_pin,
            "Int1Pin": int1_pin,
            "Int2Pin": int2_pin,
        }
        return msg


class MsgAccelUninit(MessageDefinition):
    _MSG_NAME = "AccelUninit"
    _CMD_CODE = 20
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
    _CMD_CODE = 21
    _CMD_PARAMS = [{'Name': 'DataRange', 'Type': 'uint8_t', 'Enum': 'AccelDataRange'}, {'Name': 'DataRate', 'Type': 'uint8_t', 'Enum': 'AccelDataRate'}, {'Name': 'Watermark', 'Type': 'uint8_t'}]
    _RSP_PARAMS = [{'Name': 'DataRange', 'Type': 'uint8_t', 'Enum': 'AccelDataRange'}, {'Name': 'DataRate', 'Type': 'uint8_t', 'Enum': 'AccelDataRate'}, {'Name': 'AccelData', 'Type': 'uint8_t *'}]

    @classmethod
    def builder(cls, data_range, data_rate, watermark):
        msg = cls()
        msg.parameters = {
            "DataRange": data_range,
            "DataRate": data_rate,
            "Watermark": watermark,
        }
        return msg


class MsgAccelEvents(MessageDefinition):
    _MSG_NAME = "AccelEvents"
    _CMD_CODE = 22
    _CMD_PARAMS = [{'Name': 'EnableEvents', 'Type': 'uint8_t'}, {'Name': 'DisableEvents', 'Type': 'uint8_t'}]
    _RSP_PARAMS = [{'Name': 'Enabled', 'Type': 'uint8_t'}]

    @classmethod
    def builder(cls, enable_events, disable_events):
        msg = cls()
        msg.parameters = {
            "EnableEvents": enable_events,
            "DisableEvents": disable_events,
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
    _CMD_PARAMS = [{'Name': 'Modify', 'Type': 'uint8_t', 'Enum': 'Modify', 'Default': 0}, {'Name': 'GyroModel', 'Type': 'uint8_t', 'Enum': 'SensorModel'}, {'Name': 'ChipSelectPin', 'Type': 'uint8_t', 'Description': 'Set to 0xFF for I2C mode.', 'Default': 255}, {'Name': 'Int1Pin', 'Type': 'uint8_t', 'Default': 255}, {'Name': 'Int2Pin', 'Type': 'uint8_t', 'Default': 255}]
    _RSP_PARAMS = [{'Name': 'GyroModel', 'Type': 'uint8_t', 'Enum': 'SensorModel'}, {'Name': 'CsPinControlInstance', 'Type': 'uint8_t'}, {'Name': 'Int1Pin', 'Type': 'uint8_t'}, {'Name': 'Int2Pin', 'Type': 'uint8_t'}]

    @classmethod
    def builder(cls, gyro_model, modify=0, chip_select_pin=255, int1_pin=255, int2_pin=255):
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
    _CMD_PARAMS = [{'Name': 'DataRange', 'Type': 'uint8_t', 'Enum': 'GyroDataRange'}, {'Name': 'DataRate', 'Type': 'uint8_t', 'Enum': 'GyroDataRate'}, {'Name': 'Watermark', 'Type': 'uint8_t'}]
    _RSP_PARAMS = [{'Name': 'DataRange', 'Type': 'uint8_t', 'Enum': 'GyroDataRange'}, {'Name': 'DataRate', 'Type': 'uint8_t', 'Enum': 'GyroDataRate'}, {'Name': 'GyroData', 'Type': 'uint8_t *'}]

    @classmethod
    def builder(cls, data_range, data_rate, watermark):
        msg = cls()
        msg.parameters = {
            "DataRange": data_range,
            "DataRate": data_rate,
            "Watermark": watermark,
        }
        return msg


class MsgAnalogMeasurement(MessageDefinition):
    _MSG_NAME = "AnalogMeasurement"
    _CMD_CODE = 27
    _CMD_PARAMS = [{'Name': 'Instance', 'Type': 'uint8_t'}, {'Name': 'PullConfig', 'Type': 'uint8_t', 'Enum': 'SaadcPullConfig'}, {'Name': 'Gain', 'Type': 'uint8_t', 'Enum': 'SaadcGain'}, {'Name': 'RefSel', 'Type': 'uint8_t', 'Enum': 'SaadcRefSel'}, {'Name': 'Tacq', 'Type': 'uint8_t', 'Enum': 'SaadcTacq'}]
    _RSP_PARAMS = [{'Name': 'Instance', 'Type': 'uint8_t'}, {'Name': 'Measurement', 'Type': 'int16_t'}]

    @classmethod
    def builder(cls, instance, pull_config, gain, ref_sel, tacq):
        msg = cls()
        msg.parameters = {
            "Instance": instance,
            "PullConfig": pull_config,
            "Gain": gain,
            "RefSel": ref_sel,
            "Tacq": tacq,
        }
        return msg


class MsgAnalogStream(MessageDefinition):
    _MSG_NAME = "AnalogStream"
    _CMD_CODE = 28
    _CMD_PARAMS = [{'Name': 'Instance', 'Type': 'uint8_t'}, {'Name': 'PullConfig', 'Type': 'uint8_t', 'Enum': 'SaadcPullConfig'}, {'Name': 'Gain', 'Type': 'uint8_t', 'Enum': 'SaadcGain'}, {'Name': 'RefSel', 'Type': 'uint8_t', 'Enum': 'SaadcRefSel'}, {'Name': 'Tacq', 'Type': 'uint8_t', 'Enum': 'SaadcTacq'}, {'Name': 'SamplePeriodMicroSec', 'Type': 'uint32_t'}, {'Name': 'Watermark', 'Type': 'uint8_t'}]
    _RSP_PARAMS = [{'Name': 'Instance', 'Type': 'uint8_t'}, {'Name': 'AdcData', 'Type': 'uint8_t *'}]

    @classmethod
    def builder(cls, instance, pull_config, gain, ref_sel, tacq, sample_period_micro_sec, watermark):
        msg = cls()
        msg.parameters = {
            "Instance": instance,
            "PullConfig": pull_config,
            "Gain": gain,
            "RefSel": ref_sel,
            "Tacq": tacq,
            "SamplePeriodMicroSec": sample_period_micro_sec,
            "Watermark": watermark,
        }
        return msg


class MsgRgbRunLenEnc(MessageDefinition):
    _MSG_NAME = "RgbRunLenEnc"
    _CMD_CODE = 29
    _CMD_PARAMS = [{'Name': 'Instance', 'Type': 'uint8_t'}, {'Name': 'Modify', 'Type': 'uint8_t', 'Enum': 'Modify', 'Default': 0}, {'Name': 'LedLength', 'Type': 'uint16_t'}, {'Name': 'AnimationPeriodMs', 'Type': 'uint32_t'}, {'Name': 'AnimationType', 'Type': 'uint8_t', 'Description': '0: None, 1: Rotation'}, {'Name': 'Encoding', 'Type': 'uint8_t *'}]
    _RSP_PARAMS = [{'Name': 'Instance', 'Type': 'uint8_t'}, {'Name': 'LedLength', 'Type': 'uint16_t'}, {'Name': 'AnimationRate', 'Type': 'uint8_t'}, {'Name': 'AnimationType', 'Type': 'uint8_t'}, {'Name': 'Encoding', 'Type': 'uint8_t *'}]

    @classmethod
    def builder(cls, instance, led_length, animation_period_ms, animation_type, encoding, modify=0):
        msg = cls()
        msg.parameters = {
            "Instance": instance,
            "Modify": modify,
            "LedLength": led_length,
            "AnimationPeriodMs": animation_period_ms,
            "AnimationType": animation_type,
            "Encoding": encoding,
        }
        return msg


MSG_CLASS_BY_RSP_CODE = {
    128: MsgError,
    129: MsgDeviceConfig,
    130: MsgSoftReset,
    131: MsgLoopback,
    132: MsgPinConfig,
    133: MsgPinControl,
    134: MsgPinMonitor,
    135: MsgTwimInit,
    136: MsgTwimDisconnect,
    137: MsgTwimScan,
    138: MsgTwimTransfer,
    139: MsgSpiInit,
    140: MsgSpiDisconnect,
    141: MsgSpiTransfer,
    142: MsgSpiFlashInit,
    143: MsgSpiFlashDisconnect,
    144: MsgSpiFlashErase,
    145: MsgSpiFlashRead,
    146: MsgSpiFlashWrite,
    147: MsgAccelInit,
    148: MsgAccelUninit,
    149: MsgAccelStream,
    150: MsgAccelEvents,
    151: MsgSensorEvent,
    152: MsgGyroInit,
    153: MsgGyroUninit,
    154: MsgGyroStream,
    155: MsgAnalogMeasurement,
    156: MsgAnalogStream,
    157: MsgRgbRunLenEnc,
}
