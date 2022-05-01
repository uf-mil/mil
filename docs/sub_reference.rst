Subjugator Reference
====================

Below is the reference documentation for the code on our submarine robot, SubjuGator.
The following systems are relevant only to this robot, and no other robot. However,
hyperlinks may link to systems on other robots.

Services
--------

SetValve
^^^^^^^^
.. attributetable:: sub8_actuator_board.srv._SetValve.SetValveRequest

.. class:: sub8_actuator_board.srv._SetValve.SetValveRequest

    The request class for the ``sub8_actuator_board/SetValve`` service.

    .. attribute:: actuator

        Which actuator on the sub the message references.

        :type: int

    .. attribute:: opened

        Whether the valve should be opened or closed.

        :type: bool

.. attributetable:: sub8_actuator_board.srv._SetValve.SetValveResponse

.. class:: sub8_actuator_board.srv._SetValve.SetValveResponse

    The response class for the ``sub8_actuator_board/SetValve`` service.

    .. attribute:: success

        The success of the operation.

        :type: bool

    .. attribute:: message

        If an error occurred, a message depicting the error.

        :type: bool

Exceptions
----------
.. autoclass:: sub8_actuator_board.InvalidAddressException

Actuator Board
--------------

ActuatorBoard
^^^^^^^^^^^^^
.. attributetable:: sub8_actuator_board.ActuatorBoard

.. autoclass:: sub8_actuator_board.ActuatorBoard
    :members:

ActuatorBoardSimulation
^^^^^^^^^^^^^^^^^^^^^^^
.. attributetable:: sub8_actuator_board.ActuatorBoardSimulation

.. autoclass:: sub8_actuator_board.ActuatorBoardSimulation
    :members:

CommandMessage
^^^^^^^^^^^^^^
.. attributetable:: sub8_actuator_board.CommandMessage

.. autoclass:: sub8_actuator_board.CommandMessage
    :members:

FeedbackMessage
^^^^^^^^^^^^^^^
.. attributetable:: sub8_actuator_board.FeedbackMessage

.. autoclass:: sub8_actuator_board.FeedbackMessage
    :members:
