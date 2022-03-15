Reference
=========

Messages
--------

.. class:: actionlib_msgs.msg._GoalStatus.GoalStatus

    Message type indicating the status of a goal in an actions server. Used by
    the SimpleActionClient implemented by ROS and txros' layer over it.

    The specific attributes of the message are also used in order to indicate
    a specific type of status.

    .. attribute:: PENDING

        A constant of the message type used to indicate a pending goal. Truly
        set to 0.

    .. attribute:: ACTIVE

        A constant of the message type used to indicate a active goal. Truly
        set to 1.

    .. attribute:: PREEMPTED

        A constant of the message type used to indicate a preempted goal. Truly
        set to 2.

    .. attribute:: SUCCEEDED

        A constant of the message type used to indicate a goal which succeeded. 
        Truly set to 2.

Exceptions
----------

.. currentmodule:: mil_tools

.. autoclass:: ArgumentParserException
    :members:

    An error was encountered while parsing arguments. Commonly extended by
    :class:`ThrowingArgumentParser`.

    .. attribute:: message
    
        The message associated with the error.

        :rtype: :class:`str`

Mathematics 
-------------------

Quaternion Helpers
^^^^^^^^^^^^^^^^^^
.. currentmodule:: mil_tools

.. autofunction:: mil_tools.rotate_vect_by_quat

.. autofunction:: mil_tools.skew_symmetric_cross

.. autofunction:: mil_tools.deskew

.. autofunction:: mil_tools.normalize

.. autofunction:: mil_tools.make_rotation

User Input/Output
-----------------
.. currentmodule:: mil_tools

Utility Functions
^^^^^^^^^^^^^^^^^

.. autofunction:: mil_tools.get_ch

Classes
^^^^^^^

.. autoclass:: FprintFactory
    :members:

File Input/Output
-----------------
.. currentmodule:: mil_tools

Utility Functions
^^^^^^^^^^^^^^^^^

.. autofunction:: mil_tools.download

.. autofunction:: mil_tools.download_and_unzip

Sanitization
------------
.. currentmodule:: mil_tools

Utility Functions
^^^^^^^^^^^^^^^^^

.. autofunction:: mil_tools.slugify

Images
------

Classes
^^^^^^^

.. autoclass:: BagCrawler
    :members:

.. autoclass:: CvDebug
    :members:

Serial
------

Classes
^^^^^^^

.. autoclass:: NoopSerial
    :members:

.. autoclass:: SimulatedSerial
    :members:

txros
-----

.. currentmodule:: txros

Classes
^^^^^^^

.. autoclass:: Goal
    :members:

    .. attribute:: goal

        The goal message associated with the goal.

        :rtype: :class:`GoalStatus`

    .. attribute:: status

        The status of the goal which indicates how far along the goal is to completion.

        :rtype: :class:`int`

    .. attribute:: status_text

        The string version of the goal's status.

        :rtype: :class:`str`
