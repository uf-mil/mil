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
^^^^^^^^^
All messages contain a checksum to help verify data integrity. However, receiving
packets also have a special byte containing a slightly modified checksum formula.

The checksum in all packets is found by adding up all bytes in the byte string,
including the start/end flags, and then using modulo 16 on this result.

Packet Listing
^^^^^^^^^^^^^^
Below is a listing of all available packets. The payload format is the format
used by the :mod:`struct` module. For more information, see the Python documentation
on the :ref:`list of format characters<format-characters>`, and their corresponding
byte length.

+------------+--------------+----------------+-------------------------------------------------------------------------+
| Message ID | Subclass ID  | Payload Format | Class                                                                   |
+============+==============+================+=========================================================================+
| 0x00       | 0x00         | Empty          | :class:`mil_usb_to_can.NackPacket`                                      |
+ (Meta)     +--------------+----------------+-------------------------------------------------------------------------+
|            | 0x01         | Empty          | :class:`mil_usb_to_can.AckPacket`                                       |
+------------+--------------+----------------+-------------------------------------------------------------------------+
| 0x01       | 0x00         | Empty          | :class:`sub8_thrust_and_kill_board.HeartbeatPacket`                     |
+ (Thrust/   +--------------+----------------+-------------------------------------------------------------------------+
| Kill)      | 0x01         | ``Bf``         | :class:`sub8_thrust_and_kill_board.ThrustSetPacket`                     |
+            +--------------+----------------+-------------------------------------------------------------------------+
|            | 0x02         | ``B``          | :class:`sub8_thrust_and_kill_board.KillSetPacket`                       |
+            +--------------+----------------+-------------------------------------------------------------------------+
|            | 0x03         | ``B``          | :class:`sub8_thrust_and_kill_board.KillReceivePacket`                   |
+------------+--------------+----------------+-------------------------------------------------------------------------+
| 0x02       | 0x00         | Empty          | :class:`sub8_battery_monitor_board.BatteryPollRequestPacket`            |
+ (Battery   +--------------+----------------+-------------------------------------------------------------------------+
| Monitor)   | 0x01         | ``ffff``       | :class:`sub8_battery_monitor_board.BatteryPollResponsePacket`           |
+------------+--------------+----------------+-------------------------------------------------------------------------+
| 0x03       | 0x00         | ``BB``         | :class:`sub_actuator_board.ActuatorSetPacket`                           |
+ (Actuator  +--------------+----------------+-------------------------------------------------------------------------+
| Board)     | 0x01         | Empty          | :class:`sub_actuator_board.ActuatorPollRequestPacket`                   |
+            +--------------+----------------+-------------------------------------------------------------------------+
|            | 0x02         | ``B``          | :class:`sub_actuator_board.ActuatorPollResponsePacket`                  |
+------------+--------------+----------------+-------------------------------------------------------------------------+
| 0x04       | 0x00         | Empty          | :class:`sub8_system_status_board.SetLedRequestPacket`                   |
| (System    |              |                |                                                                         |
| Status)    |              |                |                                                                         |
+------------+--------------+----------------+-------------------------------------------------------------------------+

CANDeviceHandle
^^^^^^^^^^^^^^^
.. attributetable:: mil_usb_to_can.CANDeviceHandle

.. autoclass:: mil_usb_to_can.CANDeviceHandle
    :members:

SimulatedCANDeviceHandle
^^^^^^^^^^^^^^^^^^^^^^^^
.. attributetable:: mil_usb_to_can.SimulatedCANDeviceHandle

.. autoclass:: mil_usb_to_can.SimulatedCANDeviceHandle
    :members:

Packet
^^^^^^
.. attributetable:: mil_usb_to_can.Packet

.. autoclass:: mil_usb_to_can.Packet
    :members:

Specific Packets
^^^^^^^^^^^^^^^^

NackPacket
~~~~~~~~~~
.. attributetable:: mil_usb_to_can.NackPacket

.. autoclass:: mil_usb_to_can.NackPacket
    :members:

AckPacket
~~~~~~~~~
.. attributetable:: mil_usb_to_can.AckPacket

.. autoclass:: mil_usb_to_can.AckPacket
    :members:
