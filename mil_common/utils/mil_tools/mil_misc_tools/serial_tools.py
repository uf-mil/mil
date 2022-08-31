#!/usr/bin/env python3
import binascii

import serial


def hexify(buff: bytes) -> bytes:
    """
    Display a buffer of bytes in hex format. This should take a bytes object, and
    will return a bytes string in hex format.

    .. code-block:: python3

        >>> p = CommandPacket.create_send_packet(data, can_id = can_id)
        >>> p_bytes = p.to_bytes()
        >>> print('Sending ', hexify(p_bytes))
        Sending c0:14:09:48:45:4c:4c:4f:c1

    """
    hex_form = binascii.hexlify(buff)
    return b":".join([hex_form[i : i + 2] for i in range(0, len(hex_form), 2)])


class NoopSerial(serial.Serial):
    """
    Inherits from :class:`serial.Serial`, doing nothing for each function.

    Allows super classes to implement custom behavior for simulating
    serial devices.
    """

    port = "noop-serial"

    def __init__(*args, **kwargs):
        pass

    def open(self):
        pass

    @property
    def in_waiting(self):
        return 0

    @property
    def out_waiting(self):
        return 0

    def close(self):
        pass

    def __del__(self):
        pass

    def read(self, **kwargs):
        pass

    def write(self, *args):
        pass

    def flush(self):
        pass

    def flushInput(self):
        pass

    def flushOutput(self):
        pass

    def reset_input_buffer(self):
        pass

    def reset_output_buffer(self):
        pass

    def send_break(self, *args, **kwargs):
        pass


class SimulatedSerial(NoopSerial):
    """
    Simulates a serial device, storing a buffer to be read in a program like a
    normal OS serial device.

    Intended to be extended by other classes, which should override the write function
    to receive writes to
    the simulated device. These classes simply append to the buffer string which will be returned
    on reads to the simulated device.

    Note: :class:`NoopSerial` and :class:`SimulatedSerial` are generic and are
    candidates for mil_common.

    .. container:: operations

        .. describe:: str(x)

            Pretty prints the ``SimulatedSerial`` object, its address, and the start
            of its buffer.

        .. describe:: repr(x)

            Pretty prints the ``SimulatedSerial`` object, its address, and the start
            of its buffer. Equivalent to ``str(x)``.

    Attributes:
        buffer (bytes): A buffer of bytes waiting to be read from the device.
    """

    buffer: bytes

    def __init__(self):
        self.buffer = b""

    def __str__(self) -> str:
        return f"<SimulatedSerial at 0x{id(self):0x}, buffer={self.buffer[:10]}>"

    __repr__ = __str__

    @property
    def in_waiting(self) -> int:
        """
        The number of bytes waiting to be read. This does not modify the buffer in
        any way; the buffer is only inspected.

        Returns:
            int: The count of bytes in the buffer.
        """
        return len(self.buffer)

    def reset_input_buffer(self) -> None:
        """
        Resets the buffer to contain no content. The buffer is set to an empty
        byte string.
        """
        self.buffer = b""

    def read(self, length: int) -> bytes:
        """
        Reads an amount of bytes from the buffer. All bytes read are removed from the
        buffer upon reading.

        Args:
            length (int): The amount of bytes desired to be read.

        Returns:
            bytes: The bytes originating from the top of the buffer.
        """
        b, self.buffer = self.buffer[0:length], self.buffer[length:]
        return b
