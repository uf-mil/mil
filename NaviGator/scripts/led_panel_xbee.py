#! /usr/bin/env python3
"""
Script to send a 3-series color code to the physical LED panel used at testing,
over XBee/Zigbee.
"""
import random
import time

import serial.tools.list_ports

# pip3 install digi-xbee
from digi.xbee.devices import XBeeDevice


def generate_code() -> str:
    choices = list("RGB")
    first_code = random.choice(choices)
    choices.remove(first_code)
    second_code = random.choice(choices)
    choices.append(first_code)
    choices.remove(second_code)
    third_code = random.choice(choices)
    return f"{first_code}{second_code}{third_code}"


if __name__ == "__main__":
    print("Available devices:")
    for port, _, _ in serial.tools.list_ports.comports():
        print(port)
    print()
    device_name = input("Which serial device do you want to connect to? > ")
    device = XBeeDevice(device_name, 9600)
    device.open()
    print("Device is open! Resetting... ")
    device.send_data_broadcast("off")
    time.sleep(2)
    code_to_send = input(
        'What code would you like to send (ie, "RGB")? You can also type "random". > ',
    )
    code = code_to_send if code_to_send != "random" else generate_code()
    device.send_data_broadcast(code)
    print(f'Sent "{code}"! Turning off device...')
    device.close()
