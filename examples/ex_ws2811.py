import time

from pybluemo.protocol import BluemoBlueGigaClient, PORT_PIN_E
from local_config import COM_PORT, DEVICE_NAME


def main():
    client = BluemoBlueGigaClient(port=COM_PORT)
    client.pipe_logs_to_terminal()                   # Log statements appear in the terminal output
    client.reset_ble_state()                         # In case a previous session terminated prematurely
    bluemo = client.connect_bluemo(name=DEVICE_NAME) # Find DEVICE_NAME in advertisement data and connect
    bluemo.pwr_ctrl(pwr_3v3_enable=1, pwr_main_enable=1) # Level translators require both 3.3V and 5.0V

    time.sleep(0.1)     # Give it a moment to stabilize prior to the massive load we're about to apply
    # set_ws2811 is required prior to diff_enc_ws2811 so that the attached part is configured
    bluemo.set_ws2811(PORT_PIN_E["Port A Pin 0"], length=300, rgb_values=10*b"\x00\xFF\x00")
    time.sleep(1)
    bluemo.diff_enc_ws2811(PORT_PIN_E["Port A Pin 0"], offset=10, length=10, color=b"\x10\x00\x00")
    time.sleep(1)
    bluemo.diff_enc_ws2811(PORT_PIN_E["Port A Pin 0"], offset=30, length=10, color=b"\x00\x10\x00")
    time.sleep(1)
    bluemo.diff_enc_ws2811(PORT_PIN_E["Port A Pin 0"], offset=50, length=10, color=b"\x00\x00\x10")
    time.sleep(10)

    client.disconnect(0)


if __name__ == "__main__":
    main()