Reference
=========

Messages
--------

actionlib
^^^^^^^^^

GoalStatus
~~~~~~~~~~

.. attributetable:: actionlib_msgs.msg._GoalStatus.GoalStatus

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

Alarms
^^^^^^

Alarm
~~~~~

.. attributetable:: ros_alarms.msg._Alarm.Alarm

.. class:: ros_alarms.msg._Alarm.Alarm

    A message representing a ROS Alarm.

    .. attribute:: alarm_name

        The name of the alarm.

        :type: str

    .. attribute:: raised

        Whether the alarm was raised.

        :type: bool

    .. attribute:: node_name

        The node name associated with the alarm.

        :type: str

    .. attribute:: problem_description

        The problem description associated with the alarm.

        :type: str

    .. attribute:: parameters

        The JSON parameters associated with the alarm.

        :type: str

    .. attribute:: severity

        The severity of the alarm.

        :type: int

Geometry Messages
^^^^^^^^^^^^^^^^^

Quaternion
~~~~~~~~~~

.. attributetable:: geometry_msgs.msg._Quaternion.Quaternion

.. class:: geometry_msgs.msg._Quaternion.Quaternion

    A message type representing a quaternion.

    .. attribute:: w
    
        The first element of the quaternion.

        :rtype: float

    .. attribute:: x
    
        The second element of the quaternion.

        :rtype: float

    .. attribute:: y
    
        The third element of the quaternion.

        :rtype: float

    .. attribute:: z
    
        The fourth element of the quaternion.

        :rtype: float

Point
~~~~~

.. attributetable:: geometry_msgs.msg._Point.Point

.. class:: geometry_msgs.msg._Point.Point

    A ROS message type representing a single point.

    .. attribute:: x

        The x-value of the point.

        :type: :class:`float`

    .. attribute:: y

        The y-value of the point.

        :type: :class:`float`

    .. attribute:: z

        The z-value of the point.

        :type: :class:`float`

Vector3
~~~~~~~

.. attributetable:: geometry_msgs.msg._Vector3.Vector3

.. class:: geometry_msgs.msg._Vector3.Vector3

    A ROS message type representing a three-dimensional vector.

    .. attribute:: x

        The x-value of the vector.

        :type: :class:`float`

    .. attribute:: y

        The y-value of the vector.

        :type: :class:`float`

    .. attribute:: z

        The z-value of the vector.

        :type: :class:`float`

Pose
~~~~

.. attributetable:: geometry_msgs.msg._Pose.Pose

.. class:: geometry_msgs.msg._Pose.Pose

    A ROS message type representing an object's pose.

    .. attribute:: position

        The position of the pose.

        :type: ~geometry_msgs.msg._Pose.Point

    .. attribute:: orientation

        The orientation of the pose.

        :type: ~geometry_msgs.msg._Quaternion.Quaternion

Pose2D
~~~~~~

.. attributetable:: geometry_msgs.msg._Pose2D.Pose2D

.. class:: geometry_msgs.msg._Pose2D.Pose2D

    A ROS message type representing an object's pose in two dimensions.

    .. attribute:: x

        The x-value of the pose.

        :type: float

    .. attribute:: y

        The y-value of the pose.

        :type: float

    .. attribute:: theta

        The theta value of the pose.

        :type: float

PoseWithCovariance
~~~~~~~~~~~~~~~~~~

.. attributetable:: geometry_msgs.msg._PoseWithCovariance.PoseWithCovariance

.. class:: geometry_msgs.msg._PoseWithCovariance.PoseWithCovariance

    A ROS message type representing an object's pose, along with a covariance.

    .. attribute:: pose

        The object's pose.

        :type: ~geometry_msgs.msg._Pose.Pose

    .. attribute:: covariance

        The object's covariance. Consists of a list of 36 values.

        :type: List[float]

Accel
~~~~~

.. attributetable:: geometry_msgs.msg._Accel.Accel

.. class:: geometry_msgs.msg._Accel.Accel

    A ROS message type representing an object's acceleration.

    .. attribute:: linear

        The linear acceleration of the twist.

        :type: ~geometry_msgs.msg._Vector3.Vector3

    .. attribute:: angular

        The angular acceleration of the twist.

        :type: ~geometry_msgs.msg._Vector3.Vector3

Twist
~~~~~

.. attributetable:: geometry_msgs.msg._Twist.Twist
   
.. class:: geometry_msgs.msg._Twist.Twist

    A ROS message type representing an object's twist.

    .. attribute:: linear

        The linear direction of the twist.

        :type: ~geometry_msgs.msg._Vector3.Vector3

    .. attribute:: angular

        The angular direction of the twist.

        :type: ~geometry_msgs.msg._Vector3.Vector3

TwistWithCovariance
~~~~~~~~~~~~~~~~~~~

.. attributetable:: geometry_msgs.msg._TwistWithCovariance.TwistWithCovariance

.. class:: geometry_msgs.msg._TwistWithCovariance.TwistWithCovariance

    A ROS message type representing an object's twist, along with a covariance.

    .. attribute:: twist

        The object's twist.

        :type: ~geometry_msgs.msg._Twist.Twist

    .. attribute:: covariance

        The object's covariance. Consists of a list of 36 values.

        :type: List[float]

Polygon
~~~~~~~

.. attributetable:: geometry_msgs.msg._Polygon.Polygon

.. class:: geometry_msgs.msg._Polygon.Polygon

    A ROS message type representing a polygon.

    .. attribute:: points

        The points constructing the polygon.

        :type: List[~geometry_msgs.msg._Point.Point]

Wrench
~~~~~~
        
.. attributetable:: geometry_msgs.msg._Wrench.Wrench

.. class:: geometry_msgs.msg._Wrench.Wrench

    A ROS message type representing the wrench of an object.

    .. attribute:: force

        The force associated with the object.

        :type: ~geometry_msgs.msg._Vector3.Vector3

    .. attribute:: torque

        The torque associated with the object.

        :type: ~geometry_msgs.msg._Vector3.Vector3

WrenchStamped
~~~~~~~~~~~~~

.. attributetable:: geometry_msgs.msg._WrenchStamped.WrenchStamped

.. class:: geometry_msgs.msg._WrenchStamped.WrenchStamped

    A ROS message type representing the wrench of an object with an associated header.

    .. attribute:: header

        The header associated with the message.

        :type: ~std_msgs.msg._Header.Header

    .. attribute:: wrench

        The wrench associated with the object.

        :type: ~geometry_msgs.msg._Wrench.Wrench

MIL Messages
^^^^^^^^^^^^

PoseTwist
~~~~~~~~~

.. attributetable:: mil_msgs.msg._PoseTwist.PoseTwist

.. class:: mil_msgs.msg._PoseTwist.PoseTwist

    A ROS message type representing an object's pose and twist.

    .. attribute:: pose

        The pose of the object.

        :type: ~geometry_msgs.msg._Pose.Pose

    .. attribute:: twist

        The twist of the object.

        :type: ~geometry_msgs.msg._Twist.Twist

    .. attribute:: acceleration

        The acceleration of the object.

        :type: ~geometry_msgs.msg._Accel.Accel


Navigation Messages
^^^^^^^^^^^^^^^^^^^

Odometry
~~~~~~~~

.. attributetable:: nav_msgs.msg._Odometry.Odometry

.. class:: nav_msgs.msg._Odometry.Odometry

    A ROS message type representing an object's odometry.

    .. attribute:: header

        The message header.

        :type: ~std_msgs.msg._Header.Header

    .. attribute:: child_frame_id

        The child frame ID, used to determine the frame of the robot's twist.

        :type: ~geometry_msgs.msg._Twist.Twist

    .. attribute:: pose

        The pose (along with covariance) determined within the frame of :attr:`~std_msgs.msg._Header.Header.frame_id`.

        :type: ~geometry_msgs.msg._PoseWithCovariance.PoseWithCovariance

    .. attribute:: twist

        The twist (along with covariance) determined within the frame of :attr:`~nav_msgs.msg._Odometry.Odometry.child_frame_id`.

        :type: ~geometry_msgs.msg._TwistWithCovariance.TwistWithCovariance

Standard Messages
^^^^^^^^^^^^^^^^^

Header
~~~~~~

.. attributetable:: std_msgs.msg._Header.Header

.. class:: std_msgs.msg._Header.Header

    A ROS message type representing the header of a message. Used throughout a
    variety of messages.

    .. attribute:: seq

        The sequence ID of the header. A consecutively increasing ID.

        :type: int

    .. attribute:: time

        The time of the message.

        :type: genpy.rostime.Time

    .. attribute:: frame_id

        The frame that this object is associated with.

        :type: str

ColorRGBA
~~~~~~~~~

.. attributetable:: std_msgs.msg._ColorRGBA.ColorRGBA

.. class:: std_msgs.msg._ColorRGBA.ColorRGBA

    A ROS message type representing a color with RGB and an alpha channel.

    .. attribute:: r

        The red value of the color.

        :type: float

    .. attribute:: g

        The green value of the color.

        :type: float

    .. attribute:: b

        The blue value of the color.

        :type: float

    .. attribute:: a

        The alpha value of the color.

        :type: float

Sensor Messages
^^^^^^^^^^^^^^^

PointField
~~~~~~~~~~

.. attributetable:: sensor_msgs.msg._PointField.PointField

.. class:: sensor_msgs.msg._PointField.PointField

    A ROS message type to represent a field in a point cloud.

    .. attribute:: INT8

        Constant of the data type that can be used to represent the data type of a
        value in the field. Set to ``1`` in the message definition.

        :type: int

    .. attribute:: UINT8

        Constant of the data type that can be used to represent the data type of a
        value in the field. Set to ``2`` in the message definition.

        :type: int

    .. attribute:: INT16

        Constant of the data type that can be used to represent the data type of a
        value in the field. Set to ``3`` in the message definition.

        :type: int

    .. attribute:: UINT16

        Constant of the data type that can be used to represent the data type of a
        value in the field. Set to ``4`` in the message definition.

        :type: int

    .. attribute:: INT32

        Constant of the data type that can be used to represent the data type of a
        value in the field. Set to ``5`` in the message definition.

        :type: int

    .. attribute:: UINT32

        Constant of the data type that can be used to represent the data type of a
        value in the field. Set to ``6`` in the message definition.

        :type: int

    .. attribute:: FLOAT32

        Constant of the data type that can be used to represent the data type of a
        value in the field. Set to ``7`` in the message definition.

        :type: int

    .. attribute:: FLOAT64

        Constant of the data type that can be used to represent the data type of a
        value in the field. Set to ``8`` in the message definition.

        :type: int

    .. attribute:: name

        The name of the field.

        :type: str

    .. attribute:: offset

        The offset from the start of the point struct.

        :type: int

    .. attribute:: datatype

        The datatype, represented by using one of the attributes above.

        :type: int

    .. attribute:: count

        The number of elements in the field.

        :type: int

PointCloud2
~~~~~~~~~~~

.. attributetable:: sensor_msgs.msg._PointCloud2.PointCloud2

.. class:: sensor_msgs.msg._PointCloud2.PointCloud2

    A ROS message type indicating a point cloud.

    .. attribute:: header

        The message header.

        :type: Header

    .. attribute:: height

        The height of the point cloud. If the cloud is unordered, then ``1``.

        :type: int

    .. attribute:: width

        The width of the point cloud. If the cloud is unordered, then this value
        is set to the length of the point cloud.

        :type: int

    .. attribute:: fields

        The fields in the point cloud.

        :type: List[PointField]

    .. attribute:: is_bigendian

        Whether the field is big endian.

        :type: bool

    .. attribute:: point_step

        The length of a point in bytes.

        :type: int

    .. attribute:: row_step

        The length of a row in bytes.

        :type: int

    .. attribute:: data

        The actual data inside the point cloud. The size of the array is :attr:`~sensor_msgs.msg._PointCloud2.PointCloud2.row_step`
        multiplied by :attr:`~sensor_msgs.msg._PointCloud2.PointCloud2.height`.

        :type: List[int]

    .. attribute:: is_dense

        ``True`` if there are no invalid points.

        :type: bool

Services
--------

SetBool
^^^^^^^

.. attributetable:: std_srvs.srv._SetBool.SetBoolRequest

.. class:: std_srvs.srv._SetBool.SetBoolRequest

    The request type for the ``SetBool`` service. Requests for some boolean value
    to be set.

    .. attribute:: data

        What to set the boolean as. For example ``data=False`` sets the boolean
        to be ``False``.

        :type: bool

.. attributetable:: std_srvs.srv._SetBool.SetBoolResponse

.. class:: std_srvs.srv._SetBool.SetBoolResponse

    The response type for the ``SetBool`` service. Responds to the set boolean value
    request.

    .. attribute:: success

        Whether the boolean value was succesfully set.

        :type: bool

    .. attribute:: message

        Any message included in the response.

        :type: str

KeyboardControl
^^^^^^^^^^^^^^^

.. attributetable:: navigator_msgs.srv._KeyboardControl.KeyboardControlRequest

.. class:: navigator_msgs.srv._KeyboardControl.KeyboardControlRequest

    The request class for the ``navigator_msgs/KeyboardControl`` service.

    .. attribute:: uuid

        The UUID of the request.

        :type: str

    .. attribute:: keycode

        The keycode of the key that was pressed.

        :type: int

.. attributetable:: navigator_msgs.srv._KeyboardControl.KeyboardControlResponse

.. class:: navigator_msgs.srv._KeyboardControl.KeyboardControlResponse

    The response class for the ``navigator_msgs/KeyboardControl`` service.

    .. attribute:: generated_uuid

        The generated UUID in the response class.

        :type: str

    .. attribute:: is_locked

        Whether the keyboard interface is locked.

        :type: bool

AlarmGet
^^^^^^^^

.. attributetable:: ros_alarms.srv._AlarmGet.AlarmGetRequest

.. class:: ros_alarms.srv._AlarmGet.AlarmGetRequest

   The request class for the ``ros_alarms/AlarmGet`` service.

   .. attribute:: alarm_name

        The name of the alarm to request data about.

        :type: str

.. attributetable:: ros_alarms.srv._AlarmGet.AlarmGetResponse

.. class:: ros_alarms.srv._AlarmGet.AlarmGetResponse

   The repsonse class for the ``ros_alarms/AlarmGet`` service.

   .. attribute:: header

        The header for the response.

        :type: Header

   .. attribute:: alarm

        The response data about the requested alarm.

        :type: ~ros_alarms.msg._Alarm.Alarm

AlarmSet
^^^^^^^^

.. attributetable:: ros_alarms.srv._AlarmSet.AlarmSetRequest

.. class:: ros_alarms.srv._AlarmSet.AlarmSetRequest

   The request class for the ``ros_alarms/AlarmSet`` service.

   .. attribute:: alarm

        The alarm to set.

        :type: ~ros_alarms.msg._Alarm.Alarm

.. attributetable:: ros_alarms.srv._AlarmSet.AlarmSetResponse

.. class:: ros_alarms.srv._AlarmSet.AlarmSetResponse

   The repsonse class for the ``ros_alarms/AlarmSet`` service.

   .. attribute:: succeed

        Whether the request succeeded.

        :type: bool

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

Mathematics 
-------------------

.. currentmodule:: mil_tools

.. autofunction:: mil_tools.rotate_vect_by_quat

.. autofunction:: mil_tools.skew_symmetric_cross

.. autofunction:: mil_tools.deskew

.. autofunction:: mil_tools.normalize

.. autofunction:: mil_tools.compose_transformation

.. autofunction:: mil_tools.make_rotation

.. autofunction:: mil_tools.quat_to_rotvec

.. autofunction:: mil_tools.euler_to_quat

Message Handlers
----------------
.. currentmodule:: mil_tools

.. autofunction:: pose_to_numpy

.. autofunction:: twist_to_numpy

.. autofunction:: posetwist_to_numpy

.. autofunction:: odometry_to_numpy

.. autofunction:: wrench_to_numpy

.. autofunction:: numpy_to_point

.. autofunction:: numpy_to_point2d

.. autofunction:: numpy_to_quaternion

.. autofunction:: numpy_to_twist

.. autofunction:: numpy_to_wrench

.. autofunction:: numpy_matrix_to_quaternion

.. autofunction:: numpy_pair_to_pose

.. autofunction:: numpy_quat_pair_to_pose

.. autofunction:: numpy_to_points

.. autofunction:: numpy_to_polygon

.. autofunction:: numpy_to_vector3

.. autofunction:: numpy_to_pose2D

.. autofunction:: numpy_to_colorRGBA

.. autofunction:: numpy_to_pointcloud2

.. autofunction:: make_header

.. autofunction:: make_wrench_stamped

.. autofunction:: make_pose_stamped

User Input/Output
-----------------
.. currentmodule:: mil_tools

Utility Functions
^^^^^^^^^^^^^^^^^

.. autofunction:: mil_tools.get_ch

Classes
^^^^^^^

.. attributetable:: FprintFactory

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

.. attributetable:: BagCrawler

.. autoclass:: BagCrawler
    :members:

.. attributetable:: CvDebug

.. autoclass:: CvDebug
    :members:

.. attributetable:: Image_Publisher

.. autoclass:: Image_Publisher
    :members:

.. attributetable:: Image_Subscriber

.. autoclass:: Image_Subscriber
    :members:

StereoImageSubscriber
~~~~~~~~~~~~~~~~~~~~~

.. attributetable:: StereoImageSubscriber

.. autoclass:: StereoImageSubscriber
    :members:

Plotter
~~~~~~~

.. attributetable:: Plotter

.. autoclass:: Plotter
    :members:

Serial
------

Classes
^^^^^^^

.. attributetable:: NoopSerial

.. autoclass:: NoopSerial
    :members:

.. attributetable:: SimulatedSerial

.. autoclass:: SimulatedSerial
    :members:

txros
-----

.. currentmodule:: txros

Classes
^^^^^^^

.. attributetable:: Goal

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

rviz
----

.. currentmodule:: mil_tools

Utility Functions
^^^^^^^^^^^^^^^^^

.. autofunction:: draw_sphere

.. autofunction:: draw_ray_3d

.. autofunction:: make_ray

Classes
^^^^^^^

.. attributetable:: mil_ros_tools.vector_to_marker.VectorToMarker

.. autoclass:: mil_ros_tools.vector_to_marker.VectorToMarker
    :members:

.. attributetable:: nodes.clicked_point_recorder.ClickedPointRecorder

.. autoclass:: nodes.clicked_point_recorder.ClickedPointRecorder
    :members:

.. attributetable:: nodes.network_broadcaster.NetworkBroadcaster

.. autoclass:: nodes.network_broadcaster.NetworkBroadcaster
    :members:

Threads
-------

.. currentmodule:: mil_tools

Utility Functions
^^^^^^^^^^^^^^^^^

.. autofunction:: thread_lock

Remote Control
--------------

.. attributetable:: remote_control_lib.RemoteControl

.. autoclass:: remote_control_lib.RemoteControl
    :members:

Alarms
------

Python
^^^^^^

Alarm
~~~~~
.. currentmodule:: ros_alarms

.. attributetable:: ros_alarms.Alarm

.. autoclass:: ros_alarms.Alarm
    :members:

AlarmServer
~~~~~~~~~~~
.. currentmodule:: mil_common.ros_alarms

.. attributetable:: ros_alarms.nodes.alarm_server.AlarmServer

.. autoclass:: ros_alarms.nodes.alarm_server.AlarmServer
    :members:

AlarmBroadcaster
~~~~~~~~~~~~~~~~
.. currentmodule:: ros_alarms

.. attributetable:: ros_alarms.AlarmBroadcaster

.. autoclass:: ros_alarms.AlarmBroadcaster
    :members:

AlarmListener
~~~~~~~~~~~~~
.. currentmodule:: ros_alarms

.. attributetable:: ros_alarms.AlarmListener

.. autoclass:: ros_alarms.AlarmListener
    :members:

HeartbeatMonitor
~~~~~~~~~~~~~~~~
.. currentmodule:: ros_alarms

.. attributetable:: ros_alarms.HeartbeatMonitor

.. autoclass:: ros_alarms.HeartbeatMonitor
    :members:
   
C++
^^^

AlarmProxy
~~~~~~~~~~
.. doxygenstruct:: ros_alarms::AlarmProxy

AlarmBroadcaster
~~~~~~~~~~~~~~~~
.. doxygenclass:: ros_alarms::AlarmBroadcaster

AlarmListener
~~~~~~~~~~~~~
.. doxygenclass:: ros_alarms::AlarmListener

ListenerCb
~~~~~~~~~~
.. doxygenstruct:: ros_alarms::ListenerCb

HeartbeatMonitor
~~~~~~~~~~~~~~~~
.. doxygenclass:: ros_alarms::HeartbeatMonitor

