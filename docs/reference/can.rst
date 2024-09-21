:mod:`mil_usb_to_can` - USB to CAN Communication
------------------------------------------------

.. automodule:: mil_usb_to_can

:mod:`mil_usb_to_can.sub8` - SubjuGator 8
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. automodule:: mil_usb_to_can.sub8

Packet Format
~~~~~~~~~~~~~
In order to reliably communicate with the USB to CAN board, a consistent packet format
must be defined.

Below is the USBtoCAN Packet Format. This packet format wraps every message being
sent over the serial connection to the USB to CAN board from ROS.

.. list-table:: USBtoCAN Packet Format
   :header-rows: 1

   * - Name
     - Length
     - Description
   * - Start flag (``0xC0``)
     - 1 byte
     - Flag indicating the start of a new board message
   * - Payload
     - 4-11 bytes
     - Actual data being transmitted. Often created through application packets.
       The two following tables show the payloads of command and receiving packets.
   * - End flag (``0xC1``)
     - 1 byte
     - Flag indicating end of previous board message

Below are the payload specifications for each type of transmission. Command packets
are packets sent out by the computer to complete an action (sending or requesting
information), whereas receiving packets are packets that listen to data coming from
other devices.

.. list-table:: Command Packet (:class:`.CommandPacket`)
   :header-rows: 1

   * - Name
     - Length
     - Description
   * - Length
     - 1 byte
     - Byte indicating the length of the data being sent, or the length of the data
       expected to be received, in bytes. Notably, bit 7 of this byte determines whether
       the command packet is sending a command, or receiving data through a command.
       If bit 7 is 1, then the command packet is receiving.
   * - CAN ID
     - 1 byte
     - ID of the sender, or ID of who to request from
   * - Data
     - 1-8 bytes
     - For sending command packets, the actual data being sent. For requesting command
       packets, an empty binary string.

.. list-table:: Receiving Packet (:class:`.ReceivePacket`)
   :header-rows: 1

   * - Name
     - Length
     - Description
   * - Device ID
     - 1 byte
     - The CAN ID of the device to receive data from.
   * - Payload length
     - 1 byte
     - The amount of data to listen to.
   * - Data
     - 1-8 bytes
     - The data that was received.
   * - Checksum
     - 1 byte
     - The checksum for the data.

Checksums
~~~~~~~~~
All messages contain a checksum to help verify data integrity. However, receiving
packets also have a special byte containing a slightly modified checksum formula.

The checksum in all packets is found by adding up all bytes in the byte string,
including the start/end flags, and then using modulo 16 on this result.

Exceptions
~~~~~~~~~~

Exception Hierarchy
"""""""""""""""""""
.. currentmodule:: mil_usb_to_can.sub8

.. exception_hierarchy::

    - :exc:`Exception`
        - :exc:`ApplicationPacketWrongIdentifierException`
        - :exc:`USB2CANException`
            - :exc:`ChecksumException`
            - :exc:`PayloadTooLargeException`
            - :exc:`InvalidFlagException`
                - :exc:`InvalidStartFlagException`
                - :exc:`InvalidEndFlagException`

Exception List
"""""""""""""""""""
.. autoclass:: ApplicationPacketWrongIdentifierException
    :members:

.. autoclass:: USB2CANException
    :members:

.. autoclass:: ChecksumException
    :members:

.. autoclass:: PayloadTooLargeException
    :members:

.. autoclass:: InvalidFlagException
    :members:

.. autoclass:: InvalidStartFlagException
    :members:

.. autoclass:: InvalidEndFlagException
    :members:

ApplicationPacket
~~~~~~~~~~~~~~~~~
.. attributetable:: mil_usb_to_can.sub8.ApplicationPacket

.. autoclass:: mil_usb_to_can.sub8.ApplicationPacket
    :members:

USBtoCANBoard
~~~~~~~~~~~~~
.. attributetable:: mil_usb_to_can.sub8.USBtoCANBoard

.. autoclass:: mil_usb_to_can.sub8.USBtoCANBoard
    :members:

CANDeviceHandle
~~~~~~~~~~~~~~~
.. attributetable:: mil_usb_to_can.sub8.CANDeviceHandle

.. autoclass:: mil_usb_to_can.sub8.CANDeviceHandle
    :members:

USBtoCANDriver
~~~~~~~~~~~~~~
.. attributetable:: mil_usb_to_can.sub8.USBtoCANDriver

.. autoclass:: mil_usb_to_can.sub8.USBtoCANDriver
    :members:

SimulatedCANDevice
~~~~~~~~~~~~~~~~~~
.. attributetable:: mil_usb_to_can.sub8.SimulatedCANDevice

.. autoclass:: mil_usb_to_can.sub8.SimulatedCANDevice
    :members:

SimulatedUSBtoCAN
~~~~~~~~~~~~~~~~~
.. attributetable:: mil_usb_to_can.sub8.SimulatedUSBtoCAN

.. autoclass:: mil_usb_to_can.sub8.SimulatedUSBtoCAN
    :members:

Packet
~~~~~~
.. attributetable:: mil_usb_to_can.sub8.Packet

.. autoclass:: mil_usb_to_can.sub8.Packet
    :members:

ReceivePacket
~~~~~~~~~~~~~
.. attributetable:: mil_usb_to_can.sub8.ReceivePacket

.. autoclass:: mil_usb_to_can.sub8.ReceivePacket
    :members:

CommandPacket
~~~~~~~~~~~~~
.. attributetable:: mil_usb_to_can.sub8.CommandPacket

.. autoclass:: mil_usb_to_can.sub8.CommandPacket
    :members:

:mod:`mil_usb_to_can.sub9` - SubjuGator 9
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. automodule:: mil_usb_to_can.sub9
.. currentmodule:: mil_usb_to_can.sub9

CANDeviceHandle
~~~~~~~~~~~~~~~
.. attributetable:: mil_usb_to_can.sub9.CANDeviceHandle

.. autoclass:: mil_usb_to_can.sub9.CANDeviceHandle
    :members:

SimulatedCANDeviceHandle
~~~~~~~~~~~~~~~~~~~~~~~~
.. attributetable:: mil_usb_to_can.sub9.SimulatedCANDeviceHandle

.. autoclass:: mil_usb_to_can.sub9.SimulatedCANDeviceHandle
    :members:
