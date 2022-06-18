Navigator Software Reference
============================

Below is the software reference for Navigator-specific systems.

Services
--------

MessageDetectDeliver
^^^^^^^^^^^^^^^^^^^^
.. attributetable:: navigator_msgs.srv.MessageDetectDeliverRequest

.. class:: navigator_msgs.srv.MessageDetectDeliverRequest

   The request class for the ``navigator_msgs/MessageDetectDeliver`` service.

   .. attribute:: shape_color

        The color of the requested shape.

        :type: str

   .. attribute:: shape

        The type of requested shape.

        :type: str

.. attributetable:: navigator_msgs.srv.MessageDetectDeliverResponse

.. class:: navigator_msgs.srv.MessageDetectDeliverResponse

   The repsonse class for the ``navigator_msgs/MessageDetectDeliver`` service.

   .. attribute:: message

        A message in response to the process.

        :type: str

MessageExtranceExitGate
^^^^^^^^^^^^^^^^^^^^^^^
.. attributetable:: navigator_msgs.srv.MessageExtranceExitGateRequest

.. class:: navigator_msgs.srv.MessageExtranceExitGateRequest

   The request class for the ``navigator_msgs/MessageExtranceExitGate`` service.

   .. attribute:: Extrance_gate

        The Extrance gate relevant to the task.

        :type: int

   .. attribute:: exit_gate

        The exit gate relevant to the task.

        :type: int

   .. attribute:: light_buoy_active

        Whether the task's light buoy is active.

        :type: bool

   .. attribute:: light_pattern

        The light pattern shown by the buoy. An empty string if the buoy is not active.
        In the pattern, ``R`` represents red, ``B`` represents blue, and ``G`` represents
        green.

        :type: str

.. attributetable:: navigator_msgs.srv.MessageExtranceExitGateResponse

.. class:: navigator_msgs.srv.MessageExtranceExitGateResponse

   The repsonse class for the ``navigator_msgs/MessageExtranceExitGate`` service.

   .. attribute:: message

        A message in response to the process.

        :type: str

MessageIdentifySymbolsDock
^^^^^^^^^^^^^^^^^^^^^^^^^^
.. attributetable:: navigator_msgs.srv.MessageIdentifySymbolsDockRequest

.. class:: navigator_msgs.srv.MessageIdentifySymbolsDockRequest

   The request class for the ``navigator_msgs/MessageIdentifySymbolsDock`` service.

   .. attribute:: shape_color

        The color of the shape that was identified.

        :type: str

   .. attribute:: shape

        The name of the shape that was identifed.

        :type: str


.. attributetable:: navigator_msgs.srv.MessageIdentifySymbolsDockResponse

.. class:: navigator_msgs.srv.MessageIdentifySymbolsDockResponse

   The repsonse class for the ``navigator_msgs/MessageIdentifySymbolsDock`` service.

   .. attribute:: message

        A message in response to the process.

        :type: str

ScanTheCodeMission
^^^^^^^^^^^^^^^^^^
.. attributetable:: navigator_msgs.srv.ScanTheCodeMissionRequest

.. class:: navigator_msgs.srv.ScanTheCodeMissionRequest

   The request class for the ``navigator_msgs/ScanTheCodeMission`` service.

   .. attribute:: object

        The perception object to look for.

        :type: PerceptionObject

.. attributetable:: navigator_msgs.srv.ScanTheCodeMissionResponse

.. class:: navigator_msgs.srv.ScanTheCodeMissionResponse

   The repsonse class for the ``navigator_msgs/ScanTheCodeMission`` service.

   .. attribute:: observing

        ???

        :type: bool

   .. attribute:: found

        Whether the buoy was found.

        :type: bool

   .. attribute:: colors

        The colors shown by the buoy.

        :type: List[str]

AUSVI Communication
-------------------
Below outline classes that are used to allow NaviGator to communicate with AUSVI-specific
platforms.

RobotXEntranceExitGateMessage
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
.. attributetable:: navigator_robotx_comms.RobotXEntranceExitGateMessage

.. autoclass:: navigator_robotx_comms.RobotXEntranceExitGateMessage
    :members:

RobotXDetectDeliverMessage
^^^^^^^^^^^^^^^^^^^^^^^^^^
.. attributetable:: navigator_robotx_comms.RobotXDetectDeliverMessage

.. autoclass:: navigator_robotx_comms.RobotXDetectDeliverMessage
    :members:

RobotXHeartbeatMessage
^^^^^^^^^^^^^^^^^^^^^^
.. attributetable:: navigator_robotx_comms.RobotXHeartbeatMessage

.. autoclass:: navigator_robotx_comms.RobotXHeartbeatMessage
    :members:

RobotXIdentifySymbolsDockMessage
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
.. attributetable:: navigator_robotx_comms.RobotXIdentifySymbolsDockMessage

.. autoclass:: navigator_robotx_comms.RobotXIdentifySymbolsDockMessage
    :members:

RobotXScanCodeMessage
^^^^^^^^^^^^^^^^^^^^^
.. attributetable:: navigator_robotx_comms.RobotXScanCodeMessage

.. autoclass:: navigator_robotx_comms.RobotXScanCodeMessage
    :members:

RobotXStartServices
^^^^^^^^^^^^^^^^^^^
.. attributetable:: navigator_robotx_comms.nodes.robotx_comms_client.RobotXStartServices

.. autoclass:: navigator_robotx_comms.nodes.robotx_comms_client.RobotXStartServices
    :members:

RobotXClient
^^^^^^^^^^^^
.. attributetable:: navigator_robotx_comms.nodes.robotx_comms_client.RobotXClient

.. autoclass:: navigator_robotx_comms.nodes.robotx_comms_client.RobotXClient
    :members:
