import time
from pybluemo import YaspClient, YaspBlueGigaClient, MSG_CLASS_BY_RSP_CODE
from pybluemo import MsgError, MsgPinConfig, EnumModify, EnumPinFunction
from pybluemo import MsgRgbRunLenEnc

from pybluemo.util import RgbRle

UUT = b"Bluemo v2.0"


def test_light_strip(yasp_client):
    rle = RgbRle().add_run_length(0, 50, "\x00\x00\x00")\
                  .add_run_length(10, 10, "\x7F\x00\x00")\
                  .add_run_length(20, 10, "\x00\x7F\x00")\
                  .add_run_length(30, 10, "\x00\x00\x7F")\
                  .add_run_length(40, 10, "\x7F\x7F\x00")
    msg = MsgPinConfig.builder(24, modify=EnumModify.MODIFY, function=EnumPinFunction.LED_DATA)
    resp = yasp_client.send_command(callback=None, msg_defn=msg)
    print(resp)

    inst = resp.get_param("Instance")
    msg = MsgRgbRunLenEnc.builder(inst, 300, 100000, RgbRle.ANIM_ROTATION, rle.get_bytes(), modify=EnumModify.MODIFY)
    resp = yasp_client.send_command(callback=None, msg_defn=msg)
    print(resp)


def error_handler(msg):
    print("ERROR:", msg.get_param("ErrorMessage").decode("utf-8"))


if __name__ == "__main__":
    client = YaspBlueGigaClient(port="COM3")
    client.reset_ble_state()
    yasp_client = YaspClient(MSG_CLASS_BY_RSP_CODE)
    conn_handle = client.connect_by_name(UUT, yasp_client)
    yasp_client.set_default_msg_callback(MsgError.get_response_code(), error_handler)
    time.sleep(0.1)
    test_light_strip(yasp_client)
    time.sleep(0.1)
    client.disconnect(conn_handle)
