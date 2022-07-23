Subjugator Software Reference
=============================

Below is the reference documentation for the code on our submarine robot, SubjuGator.
The following systems are relevant only to this robot, and no other robot. However,
hyperlinks may link to systems on other robots.

Messages
--------

Thrust
^^^^^^
.. attributetable:: sub8_msgs.msg.Thrust

.. class:: sub8_msgs.msg.Thrust

    Message type indicating commands for each thruster.

    .. attribute:: thruster_commands

        The commands for the thrusters.

        :type: List[:class:`~sub8_msgs.msg.ThrusterCmd`]

ThrusterCmd
^^^^^^^^^^^
.. attributetable:: sub8_msgs.msg.ThrusterCmd

.. class:: sub8_msgs.msg.ThrusterCmd

    A command for a specific thruster.

    .. attribute:: thrust

        The amount of thrust for the specific thruster.

        :type: :class:`float`

    .. attribute:: name

        The name of the thruster.

        :type: :class:`str`

Services
--------

SetValve
^^^^^^^^
.. attributetable:: sub8_actuator_board.srv.SetValveRequest

.. class:: sub8_actuator_board.srv.SetValveRequest

    The request class for the ``sub8_actuator_board/SetValve`` service.

    .. attribute:: actuator

        Which actuator on the sub the message references.

        :type: int

    .. attribute:: opened

        Whether the valve should be opened or closed.

        :type: bool

.. attributetable:: sub8_actuator_board.srv.SetValveResponse

.. class:: sub8_actuator_board.srv.SetValveResponse

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

Thrust and Kill Board
---------------------

ThrusterAndKillBoard
^^^^^^^^^^^^^^^^^^^^
.. attributetable:: sub8_thrust_and_kill_board.ThrusterAndKillBoard

.. autoclass:: sub8_thrust_and_kill_board.ThrusterAndKillBoard
    :members:

KillMessage
^^^^^^^^^^^
.. attributetable:: sub8_thrust_and_kill_board.KillMessage

.. autoclass:: sub8_thrust_and_kill_board.KillMessage
    :members:

HeartbeatMessage
^^^^^^^^^^^^^^^^
.. attributetable:: sub8_thrust_and_kill_board.HeartbeatMessage

.. autoclass:: sub8_thrust_and_kill_board.HeartbeatMessage
    :members:

ThrustPacket
^^^^^^^^^^^^
.. attributetable:: sub8_thrust_and_kill_board.ThrustPacket

.. autoclass:: sub8_thrust_and_kill_board.ThrustPacket
    :members:

ThrusterAndKillBoardSimulation
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
.. attributetable:: sub8_thrust_and_kill_board.ThrusterAndKillBoardSimulation

.. autoclass:: sub8_thrust_and_kill_board.ThrusterAndKillBoardSimulation
    :members:

Thruster
^^^^^^^^
.. attributetable:: sub8_thrust_and_kill_board.Thruster

.. autoclass:: sub8_thrust_and_kill_board.Thruster
    :members:

Object Detection
----------------

Sub8BuoyDetector
^^^^^^^^^^^^^^^^
.. cppattributetable:: Sub8BuoyDetector

.. doxygenclass:: Sub8BuoyDetector

Sub8StartGateDetector
^^^^^^^^^^^^^^^^^^^^^
.. cppattributetable:: Sub8StartGateDetector

.. doxygenclass:: Sub8StartGateDetector

Sub8TorpedoBoardDetector
^^^^^^^^^^^^^^^^^^^^^^^^
.. cppattributetable:: Sub8TorpedoBoardDetector

.. doxygenclass:: Sub8TorpedoBoardDetector

TorpedoBoardReprojectionCost
^^^^^^^^^^^^^^^^^^^^^^^^^^^^
.. cppattributetable:: TorpedoBoardReprojectionCost

.. doxygenclass:: TorpedoBoardReprojectionCost

Computer Vision
---------------

Utility Functions
^^^^^^^^^^^^^^^^^
.. doxygenfunction:: sub::voxel_filter

.. doxygenfunction:: sub::statistical_outlier_filter

.. doxygenfunction:: sub::closest_point_index_rayOMP

.. doxygenfunction:: sub::closest_point_index_ray

.. doxygenfunction:: sub::closest_point_ray

.. doxygenfunction:: sub::project_uv_to_cloud_index

.. doxygenfunction:: sub::project_uv_to_cloud

.. doxygenfunction:: sub::point_to_eigen

.. doxygenfunction:: sub::compute_normals

.. doxygenfunction:: sub::segment_rgb_region_growing

.. doxygenfunction:: sub::segment_box

.. doxygenfunction:: anisotropic_diffusion

.. doxygenfunction:: best_plane_from_combination

.. doxygenfunction:: calc_plane_coeffs

.. doxygenfunction:: point_to_plane_distance

Type Definitions
^^^^^^^^^^^^^^^^
.. doxygentypedef:: ROSCameraStream_Vec3

.. doxygentypedef:: PointNT

.. doxygentypedef:: PointCloudNT

.. doxygentypedef:: PointXYZT

.. doxygentypedef:: PointCloudT

.. doxygentypedef:: ColorHandlerT

Sub8ObjectFinder
^^^^^^^^^^^^^^^^
.. cppattributetable:: Sub8ObjectFinder

.. doxygenclass:: Sub8ObjectFinder

StereoBase
^^^^^^^^^^
.. cppattributetable:: StereoBase

.. doxygenclass:: StereoBase

RvizVisualizer
^^^^^^^^^^^^^^
.. cppattributetable:: sub::RvizVisualizer

.. doxygenclass:: sub::RvizVisualizer

OccGridUtils
^^^^^^^^^^^^
.. attributetable:: sub8_vision_tools.OccGridUtils

.. autoclass:: sub8_vision_tools.OccGridUtils
    :members:

Searcher
^^^^^^^^
.. attributetable:: sub8_vision_tools.Searcher

.. autoclass:: sub8_vision_tools.Searcher
    :members:

Classification
^^^^^^^^^^^^^^
.. cppattributetable:: Classification

.. doxygenclass:: Classification

OGridGen
^^^^^^^^
.. cppattributetable:: OGridGen

.. doxygenclass:: OGridGen

Simulation
----------
BagManager
^^^^^^^^^^
.. attributetable:: sub8_gazebo_tools.BagManager

.. autoclass:: sub8_gazebo_tools.BagManager
    :members:

6DOF Controller
---------------
Controller
^^^^^^^^^^
.. attributetable:: rise_6dof.Controller

.. autoclass:: rise_6dof.Controller
    :members:

Dynamics
--------
SubjuGatorDynamics
^^^^^^^^^^^^^^^^^^
.. attributetable:: sub8_system_id.SubjuGatorDynamics

.. autoclass:: sub8_system_id.SubjuGatorDynamics
    :members:
