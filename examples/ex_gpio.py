import time

from pybluemo.protocol import BluemoBlueGigaClient, PORT_PIN_E, PIN_CFG_E
from local_config import COM_PORT, DEVICE_NAME


def main():
    client = BluemoBlueGigaClient(port=COM_PORT)
    client.pipe_logs_to_terminal()
    client.reset_ble_state()
    bluemo = client.connect_bluemo(name=DEVICE_NAME)
    bluemo.pwr_ctrl(pwr_3v3_enable=1, pwr_3v3_off_disconnect_disable=1)
    bluemo.gpio_cfg(PORT_PIN_E["Port A Pin 0"], PIN_CFG_E["OUTPUT_HIGH"])
    time.sleep(0.1)
    bluemo.gpio_cfg(PORT_PIN_E["Port A Pin 1"], PIN_CFG_E["INPUT_NOPULL"])
    time.sleep(0.1)
    bluemo.gpio_listen(PORT_PIN_E["Port A Pin 1"], change_listen=True)
    time.sleep(2)
    for i in range(10):
        bluemo.gpio_cfg(PORT_PIN_E["Port A Pin 0"], PIN_CFG_E["OUTPUT_LOW"])
        time.sleep(2)
        bluemo.gpio_cfg(PORT_PIN_E["Port A Pin 0"], PIN_CFG_E["OUTPUT_HIGH"])
        time.sleep(2)


if __name__ == "__main__":
    main()