import struct
import time
from dataclasses import dataclass
from enum import IntEnum

from mil_tools import hexify
from mil_usb_to_can.sub9 import Packet
from serial import Serial


class KillStatus(IntEnum):
    MOBO_REQUESTED = 0
    RF_KILL = 1
    EMERGENCY_STOP = 2


@dataclass
class KillSet(Packet, msg_id=0x10, subclass_id=0x0, payload_format="?B"):
    set: bool
    status: KillStatus


@dataclass
class KillRecieve(Packet, msg_id=0x10, subclass_id=0x1, payload_format="?B"):
    set: bool
    status: KillStatus


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


device = Serial("/dev/ttyACM0", 115200, timeout=3)
try:
    for i in range(1000):
        payload = struct.pack("<?B", False, 0)
        msg = make_msg(0x37, 0x00, payload)
        p1 = bytes(KillSet(False, KillStatus.MOBO_REQUESTED))
        print("sending", hexify(p1))
        device.write(msg)
        b1 = device.read(10)
        print(b1)
        import time

        time.sleep(1)
except KeyboardInterrupt:
    print("shutting down...")
device.read(1000)
device.close()
