import time
from pybluemo import YaspClient, YaspBlueGigaClient, MSG_CLASS_BY_RSP_CODE
from pybluemo import MsgAccelEvents, MsgAccelStream, MsgSensorEvent, MsgError
from pybluemo import EnumAccelDataRange, EnumAccelDataRate
from pybluemo.sensors import Bma400StreamHandler
from pybluemo.sensordb import SensorDbClient, CloudLogger

UUT = b"Sense 840"


def test_accel_events(yasp_client):
    resp = yasp_client.send_command(callback=None, msg_defn=MsgAccelEvents.builder(1, 0))
    print(resp)
    time.sleep(600)
    resp = yasp_client.send_command(callback=None, msg_defn=MsgAccelEvents.builder(0, 1))
    print(resp)


def test_accel_stream(yasp_client):
    resp = yasp_client.send_command(callback=None, msg_defn=MsgAccelStream.builder(EnumAccelDataRange.G8, EnumAccelDataRate.F25, 7*10))
    print(resp)
    time.sleep(60)
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

    time.sleep(0.1)
    addr = "".join(["%X" % i for i in client.connections[conn_handle].address])
    sensordb_client = SensorDbClient("sensordb_client_config.json")
    cloud_logger = CloudLogger(user="Test Guy",
                               user_cohort_id="d7b50a5b-1f43-4c32-9f47-6232d48935f3",
                               device=addr, sensordb_client=sensordb_client)
    results = Bma400StreamHandler(yasp_client, "test_bma400.csv", cloud_logger)
    test_accel_stream(yasp_client)
    time.sleep(0.1)
    client.disconnect(conn_handle)
