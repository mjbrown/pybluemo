from pybluemo.protocol import BluemoBlueGigaClient
from local_config import COM_PORT, DEVICE_NAME

def test_set_ws2811_ack(bluemo):
    client = BluemoBlueGigaClient(port=COM_PORT)
    client.pipe_logs_to_terminal()
    client.reset_ble_state()
    bluemo = client.connect_bluemo(name=DEVICE_NAME)