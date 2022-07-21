:mod:`txros` - Independent extensions for ROS
---------------------------------------------

.. currentmodule:: txros

Goal
^^^^

.. attributetable:: txros.Goal

.. autoclass:: txros.Goal
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

GoalManager
^^^^^^^^^^^
.. attributetable:: txros.GoalManager

.. autoclass:: txros.GoalManager
    :members:

SimpleActionServer
^^^^^^^^^^^^^^^^^^
.. attributetable:: txros.SimpleActionServer

.. autoclass:: txros.SimpleActionServer
    :members:

ActionClient
^^^^^^^^^^^^
.. attributetable:: txros.ActionClient

.. autoclass:: txros.ActionClient
    :members:

NodeHandle
^^^^^^^^^^
.. attributetable:: txros.NodeHandle

.. autoclass:: txros.NodeHandle
    :members:

Publisher
^^^^^^^^^
.. attributetable:: txros.Publisher

.. autoclass:: txros.Publisher
    :members:

Exceptions
^^^^^^^^^^
.. attributetable:: txros.Error

.. autoclass:: txros.Error
    :members:

.. attributetable:: txros.ServiceError

.. autoclass:: txros.ServiceError
    :members:

.. attributetable:: txros.TooPastError

.. autoclass:: txros.TooPastError
    :members:

Proxy
^^^^^
.. attributetable:: txros.Proxy

.. autoclass:: txros.Proxy
    :members:

ServiceClient
^^^^^^^^^^^^^
.. attributetable:: txros.ServiceClient

.. autoclass:: txros.ServiceClient
    :members:

Service
^^^^^^^
.. attributetable:: txros.Service

.. autoclass:: txros.Service
    :members:

Subscriber
^^^^^^^^^^
.. attributetable:: txros.Subscriber

.. autoclass:: txros.Subscriber
    :members:

Transform
^^^^^^^^^^
.. attributetable:: txros.Transform

.. autoclass:: txros.Transform
    :members:

TransformBroadcaster
^^^^^^^^^^^^^^^^^^^^
.. attributetable:: txros.TransformBroadcaster

.. autoclass:: txros.TransformBroadcaster
    :members:

TransformListener
^^^^^^^^^^^^^^^^^
.. attributetable:: txros.TransformListener

.. autoclass:: txros.TransformListener
    :members:

Event
^^^^^
.. attributetable:: txros.Event

.. autoclass:: txros.Event
    :members:

Variable
^^^^^^^^
.. attributetable:: txros.Variable

.. autoclass:: txros.Variable
    :members:
