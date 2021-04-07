import time
from pybluemo import YaspClient, YaspBlueGigaClient, MSG_CLASS_BY_RSP_CODE
from pybluemo import MsgError, MsgPinConfig, EnumModify, EnumPinFunction, MsgDeviceConfig, EnumBatteryType
from pybluemo import MsgRgbRunLenEnc, MsgPinControl

from pybluemo.util import RgbRle

DEF_NAME = b"Bluemo v2.0"
UUT = b"Bluemo Airb"


def rename_device(yasp_client):
    resp = yasp_client.send_command(
        callback=None,
        msg_defn=MsgDeviceConfig.builder(UUT, EnumModify.SAVE_TO_FLASH, EnumBatteryType.NO_BATTERY, 1, 1))
    print("Device Config Response: ", resp)


def test_light_strip(yasp_client):
    msg = MsgPinConfig.builder(8, modify=EnumModify.MODIFY, function=EnumPinFunction.CONTROL)
    resp = yasp_client.send_command(callback=None, msg_defn=msg)
    inst = resp.get_param("Instance")

    msg = MsgPinControl.builder(inst, b"LED Power", EnumModify.MODIFY, drive_strength=0, output_state=1)
    resp = yasp_client.send_command(callback=None, msg_defn=msg)
    print(resp)

    rle = RgbRle().add_run_length(0, 156, "\x7F\x00\x00", rle_type=RgbRle.TYPE_UPDOWN)
    msg = MsgPinConfig.builder(24, modify=EnumModify.MODIFY, function=EnumPinFunction.LED_DATA)
    resp = yasp_client.send_command(callback=None, msg_defn=msg)
    print(resp)

    inst = resp.get_param("Instance")
    msg = MsgRgbRunLenEnc.builder(inst, 156, 10000, RgbRle.ANIM_ROTATION, rle.get_bytes(), modify=EnumModify.MODIFY)
    resp = yasp_client.send_command(callback=None, msg_defn=msg)
    print(resp)


def error_handler(msg):
    print("ERROR:", msg.get_param("ErrorMessage").decode("utf-8"))


if __name__ == "__main__":
    client = YaspBlueGigaClient(port="COM3")
    client.reset_ble_state()
    yasp_client = YaspClient(MSG_CLASS_BY_RSP_CODE)
    conn_handle = client.connect_by_name(DEF_NAME, yasp_client)
    yasp_client.set_default_msg_callback(MsgError.get_response_code(), error_handler)
    time.sleep(0.1)
    #rename_device(yasp_client)
    test_light_strip(yasp_client)
    time.sleep(0.1)
    client.disconnect(conn_handle)
