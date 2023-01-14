:mod:`mil_usb_to_can` - USB to CAN Communication
------------------------------------------------

.. automodule:: mil_usb_to_can

Packet Format
^^^^^^^^^^^^^
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
   * - Checksum
     - 1 byte
     - Checksum used to ensure that data is transmitted correctly
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

.. list-table:: Command Packet
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

.. list-table:: Receiving Packet
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

Exceptions
^^^^^^^^^^

Exception Hierarchy
~~~~~~~~~~~~~~~~~~~
.. currentmodule:: mil_usb_to_can

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
~~~~~~~~~~~~~~~~~~~
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
^^^^^^^^^^^^^^^^^
.. attributetable:: mil_usb_to_can.ApplicationPacket

.. autoclass:: mil_usb_to_can.ApplicationPacket
    :members:

USBtoCANBoard
^^^^^^^^^^^^^
.. attributetable:: mil_usb_to_can.USBtoCANBoard

.. autoclass:: mil_usb_to_can.USBtoCANBoard
    :members:

CANDeviceHandle
^^^^^^^^^^^^^^^
.. attributetable:: mil_usb_to_can.CANDeviceHandle

.. autoclass:: mil_usb_to_can.CANDeviceHandle
    :members:

USBtoCANDriver
^^^^^^^^^^^^^^
.. attributetable:: mil_usb_to_can.USBtoCANDriver

.. autoclass:: mil_usb_to_can.USBtoCANDriver
    :members:

SimulatedCANDevice
^^^^^^^^^^^^^^^^^^
.. attributetable:: mil_usb_to_can.SimulatedCANDevice

.. autoclass:: mil_usb_to_can.SimulatedCANDevice
    :members:

SimulatedUSBtoCAN
^^^^^^^^^^^^^^^^^
.. attributetable:: mil_usb_to_can.SimulatedUSBtoCAN

.. autoclass:: mil_usb_to_can.SimulatedUSBtoCAN
    :members:

Packet
^^^^^^
.. attributetable:: mil_usb_to_can.Packet

.. autoclass:: mil_usb_to_can.Packet
    :members:

ReceivePacket
^^^^^^^^^^^^^
.. attributetable:: mil_usb_to_can.ReceivePacket

.. autoclass:: mil_usb_to_can.ReceivePacket
    :members:

CommandPacket
^^^^^^^^^^^^^
.. attributetable:: mil_usb_to_can.CommandPacket

.. autoclass:: mil_usb_to_can.CommandPacket
    :members:
