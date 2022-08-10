:mod:`mil_usb_to_can` - USB to CAN Communication
------------------------------------------------

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
