NaviGator Software Reference
============================

Below is the software reference for Navigator-specific systems.

Services
--------
MessageDetectDock
^^^^^^^^^^^^^^^^^
.. attributetable:: navigator_msgs.srv.MessageDetectDockRequest

.. class:: navigator_msgs.srv.MessageDetectDockRequest

     The request class for the ``navigator_msgs/MessageDetectDock`` service.

   .. attribute:: color

        The color of the shape.

        :type: str

   .. attribute:: ams_status

        The AMS status.

        :type: int

.. attributetable:: navigator_msgs.srv.MessageDetectDockResponse

.. class:: navigator_msgs.srv.MessageDetectDockResponse

   The response class for the ``navigator_msgs/MessageDetectDock`` service.

   .. attribute:: message

        A message in response to the process.

        :type: str

MessageEntranceExitGate
^^^^^^^^^^^^^^^^^^^^^^^
.. attributetable:: navigator_msgs.srv.MessageEntranceExitGateRequest

.. class:: navigator_msgs.srv.MessageEntranceExitGateRequest

   The request class for the ``navigator_msgs/MessageEntranceExitGate`` service.

   .. attribute:: entrance_gate

        The entrance gate relevant to the task.

        :type: int

   .. attribute:: exit_gate

        The exit gate relevant to the task.

        :type: int

.. attributetable:: navigator_msgs.srv.MessageEntranceExitGateResponse

.. class:: navigator_msgs.srv.MessageEntranceExitGateResponse

   The response class for the ``navigator_msgs/MessageEntranceExitGate`` service.

   .. attribute:: message

        A message in response to the process.

        :type: str

MessageFindFling
^^^^^^^^^^^^^^^^
.. attributetable:: navigator_msgs.srv.MessageFindFlingRequest

.. class:: navigator_msgs.srv.MessageFindFlingRequest

   The request class for the ``navigator_msgs/MessageFindFling`` service.

   .. attribute:: color

        The color of the shape.

        :type: str

   .. attribute:: ams_status

        The AMS status (1=scanning, 2=flinging)

        :type: int

.. attributetable:: navigator_msgs.srv.MessageFindFlingResponse

.. class:: navigator_msgs.srv.MessageFindFlingResponse

   The response class for the ``navigator_msgs/MessageFindFling`` service.

   .. attribute:: message

        A message in response to the process.

        :type: str

MessageFollowPath
^^^^^^^^^^^^^^^^^
.. attributetable:: navigator_msgs.srv.MessageFollowPathRequest

.. class:: navigator_msgs.srv.MessageFollowPathRequest

   The request class for the ``navigator_msgs/MessageFollowPath`` service.

   .. attribute:: finished

        The bool to say if we are finished following the path. (1=in progress 2=completed)

        :type: int

.. attributetable:: navigator_msgs.srv.MessageFollowPathResponse

.. class:: navigator_msgs.srv.MessageFollowPathResponse

   The response class for the ``navigator_msgs/MessageFollowPath`` service.

   .. attribute:: message

        A message in response to the process.

        :type: str

MessageReactReport
^^^^^^^^^^^^^^^^^^
.. attributetable:: navigator_msgs.srv.MessageReactReportRequest

.. class:: navigator_msgs.srv.MessageReactReportRequest

   The request class for the ``navigator_msgs/MessageReactReport`` service.

   .. attribute:: animal_array

        List of animals (P,C,T), up to three animals (Platypus, Turtle, Croc)

        :type: string[]

.. attributetable:: navigator_msgs.srv.MessageReactReportResponse

.. class:: navigator_msgs.srv.MessageReactReportResponse

   The response class for the ``navigator_msgs/MessageReactReport`` service.

   .. attribute:: message

        A message in response to the process.

        :type: str

MessageUAVReplenishment
^^^^^^^^^^^^^^^^^^^^^^^
.. attributetable:: navigator_msgs.srv.MessageUAVReplenishmentRequest

.. class:: navigator_msgs.srv.MessageUAVReplenishmentRequest

   The request class for the ``navigator_msgs/MessageUAVReplenishment`` service.

   .. attribute:: uav_status

        The UAV status # 1=stowed, 2=deployed, 3=faulted

        :type: int

   .. attribute:: item_status

        The item status # 0=not picked up, 1=picked up, 2=delivered

        :type: int

.. attributetable:: navigator_msgs.srv.MessageUAVReplenishmentResponse

.. class:: navigator_msgs.srv.MessageUAVReplenishmentResponse

   The response class for the ``navigator_msgs/MessageUAVReplenishment`` service.

   .. attribute:: message

        A message in response to the process.

        :type: str

MessageUAVSearchReport
^^^^^^^^^^^^^^^^^^^^^^
.. attributetable:: navigator_msgs.srv.MessageUAVSearchReportRequest

.. class:: navigator_msgs.srv.MessageUAVSearchReportRequest

   The request class for the ``navigator_msgs/MessageUAVSearchReport`` service.

   .. attribute:: object1

        The object found (R or N)

        :type: string

   .. attribute:: object1_latitude

        The latitude of object 1

        :type: float64

   .. attribute:: object1_n_s

        The N/S of object 1

        :type: string

   .. attribute:: object1_longitude

        The longitude of object 1

        :type: float64

   .. attribute:: object1_e_w

        The E/W of object 1

        :type: string

   .. attribute:: object2

        The object found (R or N)

        :type: string

   .. attribute:: object2_latitude

        The latitude of object 2

        :type: float64

   .. attribute:: object2_n_s

        The N/S of object 2

        :type: string

   .. attribute:: object2_longitude

        The longitude of object 2

        :type: float64

   .. attribute:: object2_e_w

        The E/W of object 2

        :type: string

   .. attribute:: uav_status

        The UAV status # 1=manual, 2=autonomous, 3=faulted

        :type: int

.. attributetable:: navigator_msgs.srv.MessageUAVSearchReportResponse

.. class:: navigator_msgs.srv.MessageUAVSearchReportResponse

   The response class for the ``navigator_msgs/MessageUAVSearchReport`` service.

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

   The response class for the ``navigator_msgs/ScanTheCodeMission`` service.

   .. attribute:: observing

        ???

        :type: bool

   .. attribute:: found

        Whether the buoy was found.

        :type: bool

   .. attribute:: colors

        The colors shown by the buoy.

        :type: List[str]


AUVSI Communication
-------------------
Below outline classes that are used to allow NaviGator to communicate with AUVSI-specific
platforms.

RobotXEntranceExitGateMessage
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
.. attributetable:: navigator_robotx_comms.RobotXEntranceExitGateMessage

.. autoclass:: navigator_robotx_comms.RobotXEntranceExitGateMessage
    :members:

RobotXDetectDockMessage
^^^^^^^^^^^^^^^^^^^^^^^
.. attributetable:: navigator_robotx_comms.RobotXDetectDockMessage

.. autoclass:: navigator_robotx_comms.RobotXDetectDockMessage
    :members:

RobotXFindFlingMessage
^^^^^^^^^^^^^^^^^^^^^^
.. attributetable:: navigator_robotx_comms.RobotXFindFlingMessage

.. autoclass:: navigator_robotx_comms.RobotXFindFlingMessage
    :members:

RobotXFollowPathMessage
^^^^^^^^^^^^^^^^^^^^^^^
.. attributetable:: navigator_robotx_comms.RobotXFollowPathMessage

.. autoclass:: navigator_robotx_comms.RobotXFollowPathMessage
    :members:

RobotXHeartbeatMessage
^^^^^^^^^^^^^^^^^^^^^^
.. attributetable:: navigator_robotx_comms.RobotXHeartbeatMessage

.. autoclass:: navigator_robotx_comms.RobotXHeartbeatMessage
    :members:

RobotXReactReportMessage
^^^^^^^^^^^^^^^^^^^^^^^^
.. attributetable:: navigator_robotx_comms.RobotXReactReportMessage

.. autoclass:: navigator_robotx_comms.RobotXReactReportMessage
    :members:

RobotXScanCodeMessage
^^^^^^^^^^^^^^^^^^^^^
.. attributetable:: navigator_robotx_comms.RobotXScanCodeMessage

.. autoclass:: navigator_robotx_comms.RobotXScanCodeMessage
    :members:

RobotXUAVReplenishmentMessage
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
.. attributetable:: navigator_robotx_comms.RobotXUAVReplenishmentMessage

.. autoclass:: navigator_robotx_comms.RobotXUAVReplenishmentMessage
    :members:

RobotXUAVSearchReportMessage
^^^^^^^^^^^^^^^^^^^^^^^^^^^^
.. attributetable:: navigator_robotx_comms.RobotXUAVSearchReportMessage

.. autoclass:: navigator_robotx_comms.RobotXUAVSearchReportMessage
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

:mod:`navigator_ball_launcher` - Ball launcher
----------------------------------------------

.. automodule:: navigator_ball_launcher
    :members:

ReleaseBallPacket
^^^^^^^^^^^^^^^^^
.. attributetable:: navigator_ball_launcher.ReleaseBallPacket

.. autoclass:: navigator_ball_launcher.ReleaseBallPacket
    :members:

SetSpinPacket
^^^^^^^^^^^^^
.. attributetable:: navigator_ball_launcher.SetSpinPacket

.. autoclass:: navigator_ball_launcher.SetSpinPacket
    :members:

:mod:`navigator_drone_comm` - Boat-drone communication standard
---------------------------------------------------------------

.. automodule:: navigator_drone_comm
    :members:

HeartbeatReceivePacket
^^^^^^^^^^^^^^^^^^^^^^
.. attributetable:: navigator_drone_comm.HeartbeatReceivePacket

.. autoclass:: navigator_drone_comm.HeartbeatReceivePacket
    :members:

HeartbeatSetPacket
^^^^^^^^^^^^^^^^^^
.. attributetable:: navigator_drone_comm.HeartbeatSetPacket

.. autoclass:: navigator_drone_comm.HeartbeatSetPacket
    :members:

GPSDronePacket
^^^^^^^^^^^^^^
.. attributetable:: navigator_drone_comm.GPSDronePacket

.. autoclass:: navigator_drone_comm.GPSDronePacket
    :members:

EStopPacket
^^^^^^^^^^^
.. attributetable:: navigator_drone_comm.EStopPacket

.. autoclass:: navigator_drone_comm.EStopPacket
    :members:

StopPacket
^^^^^^^^^^
.. attributetable:: navigator_drone_comm.StopPacket

.. autoclass:: navigator_drone_comm.StopPacket
    :members:

StartPacket
^^^^^^^^^^^
.. attributetable:: navigator_drone_comm.StartPacket

.. autoclass:: navigator_drone_comm.StartPacket
    :members:

Color
^^^^^
.. attributetable:: navigator_drone_comm.Color

.. autoclass:: navigator_drone_comm.Color
    :members:

TargetPacket
^^^^^^^^^^^^
.. attributetable:: navigator_drone_comm.TargetPacket

.. autoclass:: navigator_drone_comm.TargetPacket
    :members:
