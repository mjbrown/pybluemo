import json
import os

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

with open(os.path.dirname(__file__) + "/yasp_specification.json", "r") as _yasp_fp:
    PROTOCOL_JSON = json.load(_yasp_fp)
    PROTOCOL_VERSION = PROTOCOL_JSON["ProtocolVersion"]
    MESSAGES = PROTOCOL_JSON[MSGS]

MESSAGES_BY_NAME = {}

for _msg in range(len(MESSAGES)):
    MESSAGES_BY_NAME[MESSAGES[_msg][MSG_NAME]] = MESSAGES[_msg]
    MESSAGES_BY_NAME[MESSAGES[_msg][MSG_NAME]][MSG_CODE] = _msg

DATA_TYPE_SIZES = {
    "int8_t": 1,
    "uint8_t": 1,
    "int16_t": 2,
    "uint16_t": 2,
    "uint32_t": 4,
    "float": 4,
    "uint8_t *": 0
}
