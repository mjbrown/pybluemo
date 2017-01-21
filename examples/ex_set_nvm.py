import time

from pybluemo.protocol import BluemoBlueGigaClient, NVM_BLOCK_E
from local_config import COM_PORT


def main():
    client = BluemoBlueGigaClient(port=COM_PORT)
    client.pipe_logs_to_terminal()
    client.reset_ble_state()
    bluemo = client.connect_bluemo(name=b"Bluemo NoInit")
    bluemo.set_nvm_block(NVM_BLOCK_E["HW_REV_BLOCK_ID"], b"LiFePO4 5V NRF51-8")
    time.sleep(1)
    bluemo.set_nvm_block(NVM_BLOCK_E["SERIAL_BLOCK_ID"], b"05")
    time.sleep(1)
    bluemo.set_nvm_block(NVM_BLOCK_E["DEV_NAME_BLOCK_ID"], b"Bluemo 05")
    time.sleep(1)

    client.disconnect(0)


if __name__ == "__main__":
    main()