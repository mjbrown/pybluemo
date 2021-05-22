import time
from pybluemo import YaspClient, YaspBlueGigaClient, MSG_CLASS_BY_RSP_CODE
from pybluemo import MsgAccelEvents, MsgAccelStream, MsgSensorEvent, MsgError
from pybluemo import EnumAccelDataRange, EnumAccelDataRate
from pybluemo.sensors import Bma400StreamHandler

from pybluemo import MsgSpiFlashInit, MsgSpiFlashRead, MsgSpiFlashDisconnect, MsgSpiFlashWrite, MsgSpiFlashErase
from pybluemo import EnumSpiFreq, EnumFlashModel

UUT = b"Sense 840"


class AccelStreamLogger(Bma400StreamHandler):
    def parsed_data_ready(self, x, y, z, t):
        if self.csv_fp is not None:
            self.csv_fp.write("%s,%f,%f,%f\n" % (t, x, y, z))
        print("Parsed X:%f Y:%f Z:%f" % (x, y, z))


def test_spi_flash(yasp_client):
    TEST_ADDR = 1024*64
    TEST_LENGTH = 256
    TEST_DATA = b"\xA5" * TEST_LENGTH
    time.sleep(1)
    resp = yasp_client.send_command(callback=None, msg_defn=MsgSpiFlashInit.builder())
    print(resp)
    cs_inst = resp.get_param("CsPinControlInstance")
    if resp.get_param("FlashModel") != 1:
        raise Exception("Flash unresponsive.")
    time.sleep(1)
    resp = yasp_client.send_command(callback=None, msg_defn=MsgSpiFlashRead.builder(cs_inst, TEST_ADDR, TEST_LENGTH))
    print(resp)
    resp = yasp_client.send_command(callback=None, msg_defn=MsgSpiFlashErase.builder(cs_inst, TEST_ADDR, 4096))
    print(resp)
    resp = yasp_client.send_command(callback=None, msg_defn=MsgSpiFlashRead.builder(cs_inst, TEST_ADDR, TEST_LENGTH))
    print(resp)
    resp = yasp_client.send_command(callback=None, msg_defn=MsgSpiFlashWrite.builder(cs_inst, TEST_ADDR, TEST_DATA))
    print(resp)
    resp = yasp_client.send_command(callback=None, msg_defn=MsgSpiFlashRead.builder(cs_inst, TEST_ADDR, TEST_LENGTH))
    print(resp)
    #resp = yasp_client.send_command(callback=None, msg_defn=MsgSpiFlashDisconnect.builder())
    #print(resp)
    time.sleep(1)


def test_accel_events(yasp_client):
    resp = yasp_client.send_command(callback=None, msg_defn=MsgAccelEvents.builder(1, 0))
    print(resp)
    time.sleep(600)
    resp = yasp_client.send_command(callback=None, msg_defn=MsgAccelEvents.builder(0, 1))
    print(resp)


def test_accel_stream(yasp_client):
    resp = yasp_client.send_command(callback=None, msg_defn=MsgAccelStream.builder(EnumAccelDataRange.G8, EnumAccelDataRate.F200, 7*10))
    print(resp)
    time.sleep(10)
    resp = yasp_client.send_command(callback=None, msg_defn=MsgAccelStream.builder(EnumAccelDataRange.G4, EnumAccelDataRate.OFF, 7*10))
    print(resp)


def error_handler(msg):
    print("ERROR:", msg.get_param("ErrorMessage").decode("utf-8"))


if __name__ == "__main__":
    client = YaspBlueGigaClient(port="COM3")
    client.reset_ble_state()
    #client.pipe_logs_to_terminal()
    yasp_client = YaspClient(MSG_CLASS_BY_RSP_CODE)
    conn_handle = client.connect_by_name(UUT, yasp_client)
    yasp_client.set_default_msg_callback(MsgError.get_response_code(), error_handler)

    #results = Bma400StreamHandler(yasp_client, "test_bma400.csv")
    time.sleep(0.1)
    #test_accel_events(yasp_client)
    #test_accel_stream(yasp_client)
    test_spi_flash(yasp_client)
    time.sleep(0.1)
    client.disconnect(conn_handle)
