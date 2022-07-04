Exceptions
----------

.. currentmodule:: mil_tools

.. attributetable:: ArgumentParserException

.. autoclass:: ArgumentParserException
    :members:

    An error was encountered while parsing arguments. Commonly extended by
    :class:`ThrowingArgumentParser`.

    .. attribute:: message
    
        The message associated with the error.

        :rtype: :class:`str`

.. currentmodule:: mil_usb_to_can

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
