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
   * - Sync character 1 (``0x37``)
     - 1 byte
     - First sync character indicating the start of packets.
   * - Sync character 2 (``0x01``)
     - 1 byte
     - Second sync character indicating the start of packets.
   * - Class ID
     - 1 byte
     - Message class. Determines the family of messages the packet belongs to.
   * - Subclass ID
     - 1 byte
     - Message subclass. In combination with class, determines specific qualities
       of message.
   * - Payload Length
     - 2 bytes
     - Length of payload.
   * - Payload
     - 0-65535 bytes
     - Payload. Meaning of payload is determined by specific packet class/subclass.
   * - Checksum A
     - 1 byte
     - First byte of Fletcher's checksum.
   * - Checksum B
     - 1 byte
     - Second byte of Fletcher's checksum.

Checksums
~~~~~~~~~
All messages contain a checksum to help verify data integrity. However, receiving
packets also have a special byte containing a slightly modified checksum formula.

The checksum in all packets is found by adding up all bytes in the byte string,
including the start/end flags, and then using modulo 16 on this result.

Packet Listing
~~~~~~~~~~~~~~
Below is a listing of all available packets. The payload format is the format
used by the :mod:`struct` module. For more information, see the Python documentation
on the :ref:`list of format characters<format-characters>`, and their corresponding
byte length.

+------------+--------------+----------------+-------------------------------------------------------------------------+
| Message ID | Subclass ID  | Payload Format | Class                                                                   |
+============+==============+================+=========================================================================+
| 0x00       | 0x00         | Empty          | :class:`mil_usb_to_can.sub9.NackPacket`                                 |
+ (Meta)     +--------------+----------------+-------------------------------------------------------------------------+
|            | 0x01         | Empty          | :class:`mil_usb_to_can.sub9.AckPacket`                                  |
+------------+--------------+----------------+-------------------------------------------------------------------------+
| 0x01       | 0x00         | Empty          | :class:`sub8_thrust_and_kill_board.HeartbeatPacket`                     |
+ (Sub8      +--------------+----------------+-------------------------------------------------------------------------+
| Thrust/    | 0x01         | ``Bf``         | :class:`sub8_thrust_and_kill_board.ThrustSetPacket`                     |
+ Kill)      +--------------+----------------+-------------------------------------------------------------------------+
|            | 0x02         | ``B``          | :class:`sub8_thrust_and_kill_board.KillSetPacket`                       |
+            +--------------+----------------+-------------------------------------------------------------------------+
|            | 0x03         | ``B``          | :class:`sub8_thrust_and_kill_board.KillReceivePacket`                   |
+------------+--------------+----------------+-------------------------------------------------------------------------+
| 0x02       | 0x00         | Empty          | :class:`sub9_thrust_and_kill_board.HeartbeatSetPacket`                  |
+ (Sub9      +--------------+----------------+-------------------------------------------------------------------------+
| Thrust/    | 0x01         | Empty          | :class:`sub9_thrust_and_kill_board.HeartbeatReceivePacket`              |
+ Kill)      +--------------+----------------+-------------------------------------------------------------------------+
|            | 0x02         | ``Bf``         | :class:`sub9_thrust_and_kill_board.ThrustSetPacket`                     |
+            +--------------+----------------+-------------------------------------------------------------------------+
|            | 0x03         | ``B``          | :class:`sub9_thrust_and_kill_board.KillSetPacket`                       |
+            +--------------+----------------+-------------------------------------------------------------------------+
|            | 0x04         | ``B``          | :class:`sub9_thrust_and_kill_board.KillReceivePacket`                   |
+------------+--------------+----------------+-------------------------------------------------------------------------+
| 0x03       | 0x00         | Empty          | :class:`sub8_battery_monitor_board.BatteryPollRequestPacket`            |
+ (Battery   +--------------+----------------+-------------------------------------------------------------------------+
| Monitor)   | 0x01         | ``ffff``       | :class:`sub8_battery_monitor_board.BatteryPollResponsePacket`           |
+------------+--------------+----------------+-------------------------------------------------------------------------+
| 0x04       | 0x00         | ``BB``         | :class:`sub_actuator_board.ActuatorSetPacket`                           |
+ (Actuator  +--------------+----------------+-------------------------------------------------------------------------+
| Board)     | 0x01         | Empty          | :class:`sub_actuator_board.ActuatorPollRequestPacket`                   |
+            +--------------+----------------+-------------------------------------------------------------------------+
|            | 0x02         | ``B``          | :class:`sub_actuator_board.ActuatorPollResponsePacket`                  |
+------------+--------------+----------------+-------------------------------------------------------------------------+
| 0x05       | 0x00         | Empty          | :class:`sub9_system_status_board.SetLedRequestPacket`                   |
| (System    |              |                |                                                                         |
| Status)    |              |                |                                                                         |
+------------+--------------+----------------+-------------------------------------------------------------------------+

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

Packet
~~~~~~
.. attributetable:: mil_usb_to_can.sub9.Packet

.. autoclass:: mil_usb_to_can.sub9.Packet
    :members:

NackPacket
~~~~~~~~~~
.. attributetable:: mil_usb_to_can.sub9.NackPacket

.. autoclass:: mil_usb_to_can.sub9.NackPacket
    :members:

AckPacket
~~~~~~~~~
.. attributetable:: mil_usb_to_can.sub9.AckPacket

.. autoclass:: mil_usb_to_can.sub9.AckPacket
    :members:
