Subjugator Software Reference
=============================

Below is the reference documentation for the code on our submarine robot, SubjuGator.
The following systems are relevant only to this robot, and no other robot. However,
hyperlinks may link to systems on other robots.

Messages
--------

Thrust
^^^^^^
.. attributetable:: subjugator_msgs.msg.Thrust

.. class:: subjugator_msgs.msg.Thrust

    Message type indicating commands for each thruster.

    .. attribute:: thruster_commands

        The commands for the thrusters.

        :type: List[:class:`~subjugator_msgs.msg.ThrusterCmd`]

ThrusterCmd
^^^^^^^^^^^
.. attributetable:: subjugator_msgs.msg.ThrusterCmd

.. class:: subjugator_msgs.msg.ThrusterCmd

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

Thrust and Kill Board
---------------------

ThrusterAndKillBoard
^^^^^^^^^^^^^^^^^^^^
.. attributetable:: sub8_thrust_and_kill_board.ThrusterAndKillBoard

.. autoclass:: sub8_thrust_and_kill_board.ThrusterAndKillBoard
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

SubjuGatorBuoyDetector
^^^^^^^^^^^^^^^^^^^^^^
.. cppattributetable:: SubjuGatorBuoyDetector

.. doxygenclass:: SubjuGatorBuoyDetector

SubjuGatorStartGateDetector
^^^^^^^^^^^^^^^^^^^^^^^^^^^
.. cppattributetable:: SubjuGatorStartGateDetector

.. doxygenclass:: SubjuGatorStartGateDetector

SubjuGatorTorpedoBoardDetector
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
.. cppattributetable:: SubjuGatorTorpedoBoardDetector

.. doxygenclass:: SubjuGatorTorpedoBoardDetector

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

SubjuGatorObjectFinder
^^^^^^^^^^^^^^^^^^^^^^
.. cppattributetable:: SubjuGatorObjectFinder

.. doxygenclass:: SubjuGatorObjectFinder

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
.. attributetable:: subjugator_vision_tools.OccGridUtils

.. autoclass:: subjugator_vision_tools.OccGridUtils
    :members:

Searcher
^^^^^^^^
.. attributetable:: subjugator_vision_tools.Searcher

.. autoclass:: subjugator_vision_tools.Searcher
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
.. attributetable:: subjugator_gazebo_tools.BagManager

.. autoclass:: subjugator_gazebo_tools.BagManager
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
.. attributetable:: subjugator_system_id.SubjuGatorDynamics

.. autoclass:: subjugator_system_id.SubjuGatorDynamics
    :members:

Missions
--------
PoseEditor
^^^^^^^^^^
.. attributetable:: subjugator_missions.PoseEditor

.. autoclass:: subjugator_missions.PoseEditor
    :members:
