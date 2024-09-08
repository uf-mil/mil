#!/usr/bin/env python3

"""Code meant to connect to servo tube and do test with"""

import struct

import serial

# ---------------------------------------------------------------------------- #
#                                   FUNCTIONS                                  #
# ---------------------------------------------------------------------------- #


def fletcher16(data: bytes):
    """Compute the Fletcher-16 checksum of a byte sequence."""
    sum1 = 0
    sum2 = 0
    for index, byte in enumerate(data):
        sum1 = (sum1 + byte) % 255
        sum2 = (sum2 + sum1) % 255
    return (sum2 << 8) | sum1


def make_msg(class_id: int, subclass_id: int, payload: bytes) -> bytes:
    """
    Create a message packet to send via usb.

    Inputs:
    - class_id (int): the device that it is being sent to (check uf-mil.github.io page or USB-TO-CAN.H)
    - subclass_id (int): what operation is the device doing (check the github.io or .h file)
    - playload (byte): message being sent (again check the github.io or .h file)

    Returns
    - bytes: full message byte array
    """
    sync = b"\x37\x01"
    cid_byte = struct.pack("<B", class_id)
    scid_byte = struct.pack("<B", subclass_id)
    length = struct.pack("<H", len(payload))

    # get the current msg to find checksum
    data1 = cid_byte + scid_byte + length + payload

    checksum = fletcher16(data1)
    checksum_bytes = struct.pack("<H", checksum)

    data = sync + data1 + checksum_bytes

    return data


def msg_to_string(msg):
    """Convert a message to a string to print."""
    return " ".join(f"{byte:02x}" for byte in msg)


# ---------------------------------------------------------------------------- #
#                                     MAIN                                     #
# ---------------------------------------------------------------------------- #

# Connect to device
# ON WINDOWS, CHANGE THIS TO COM# , /dev/ is chad linux
ser = serial.Serial("/dev/ttyACM0", 115200, timeout=1)

# pre-declared variables cause I am lazy
class_id = 0x10
subclass_id = 0x00

# -------------------------------- subclasses -------------------------------- #
acceptable_subclasses = [0, 1, 69]
acceptable_kill = [0, 1]
help_id = 69


help_msg = f"""
RF Kill Controller
Class ID: {class_id}
Subclasses:
    - 0x00 : Kill Set
    - 0x01 : Kill Received (DO NOT USE)

To repeat this message use subclass id 69
    """

print("Starting Temp RF Kill Board test file...\n", help_msg)

# main loop
while True:
    try:
        subclass_id = int(input("Enter id (type 69 for help): "))

        while subclass_id not in acceptable_subclasses:
            subclass_id = int(input("Invalid option. Enter id (69 for help): "))

        if subclass_id == 0:

            toKill = int(input("Do you want to kill (1) or unkill (0): "))

            while toKill not in acceptable_kill:
                toKill = int(input("Invalid option, Kill (1) or Unkill (0): "))

            if toKill:
                payload = struct.pack("<?B", True, 0)
            else:
                payload = struct.pack("<?B", False, 0)

            msg = make_msg(class_id, subclass_id, payload)
            print(msg)

            print(f"Message send: {msg_to_string(msg)}")
            ser.write(msg)

        elif subclass_id == help_id:
            print(help_msg)
            continue

        # check if ack or nak
        response = ser.read(10)
        print(f"Received the message {msg_to_string(response)}")

    except KeyboardInterrupt:
        print("\nKeyboard Interrupt has occurred! Ending program now...")
        break
    except Exception as e:
        print(e)

ser.read(1000)  # read any messages that were received
ser.close()
