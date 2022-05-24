Software Reference
==================

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

Acceleration
~~~~~~~~~~~~

.. attributetable:: navigator_msgs.msg._Acceleration.Acceleration

.. class:: navigator_msgs.msg._Acceleration.Acceleration

    .. attribute:: linear

        The linear component of the acceleration.

        :type: Vector3

    .. attribute:: angular

        The angular component of the acceleration.

        :type: Vector3

KillStatus
~~~~~~~~~~

.. attributetable:: navigator_msgs.msg._KillStatus.KillStatus

.. class:: navigator_msgs.msg._KillStatus.KillStatus

    A custom message to represent information about a kill induced on the robot.

    .. attribute:: overall

        :type: bool

    .. attribute:: pf

        :type: bool

    .. attribute:: pa

        :type: bool

    .. attribute:: sf

        :type: bool

    .. attribute:: sa

        :type: bool

    .. attribute:: remote

        :type: bool

    .. attribute:: computer

        :type: bool

    .. attribute:: remote_conn

        :type: bool

PerceptionObject
~~~~~~~~~~~~~~~~

.. attributetable:: navigator_msgs.msg._PerceptionObject.PerceptionObject

.. class:: navigator_msgs.msg._PerceptionObject.PerceptionObject

    A custom message definition to represent an object found by the perception
    system.

    .. attribute:: header
    
        The message header.

        :type: Header

    .. attribute:: name
    
        The name of the object.

        :type: str

    .. attribute:: DETECT_DELIVER_PLATFORM
    
        The constant string field representing the platform to detect and deliver.
        Actual string value is ``shooter``.

        :type: str

    .. attribute:: IDENTIFY_AND_DOCK
    
        The constant string field representing the dock in the Identify and Dock mission.
        Actual string value is ``dock``.

        :type: str

    .. attribute:: SCAN_THE_CODE
    
        The constant string field representing the Scan the Code totem. Actual 
        string value is ``scan_the_code``.

        :type: str

    .. attribute:: TOTEM
    
        The constant string field representing the totem in the Find Totems mission.
        Actual string value is ``totem``.

        :type: str

    .. attribute:: START_GATE_BUOY
    
        The constant string field representing the buoy of the start gate.
        Actual string value is ``start_gate``.

        :type: str

    .. attribute:: BUOY
    
        The constant string field representing a buoy.
        Actual string value is ``buoy``.

        :type: str

    .. attribute:: UNKNOWN
    
        The constant string field representing an unknown object.
        Actual string value is ``unknown``.

        :type: str

    .. attribute:: ALL
    
        The constant string field representing all objects found.
        Actual string value is ``all``.

        :type: str

    .. attribute:: FAKE_SHOOTER
    
        The constant string field representing a fake shooter.
        Actual string value is ``Shooter``.

        :type: str

    .. attribute:: FAKE_IDENTIFY_AND_DOCK
    
        The constant string field representing a fake dock in the Identify and Dock mission.
        Actual string value is ``Dock``.

        :type: str

    .. attribute:: FAKE_SCAN_THE_CODE
    
        The constant string field representing a fake Scan the Code totem in the
        Scan the Code mission. Actual string value is ``Dock``.

        :type: str

    .. attribute:: GATE1
    
        The constant string field representing the first gate.
        Actual string value is ``Gate_1``.

        :type: str

    .. attribute:: GATE2
    
        The constant string field representing the first gate.
        Actual string value is ``Gate_2``.

        :type: str

    .. attribute:: GATE3
    
        The constant string field representing the first gate.
        Actual string value is ``Gate_3``.

        :type: str

    .. attribute:: BUOY_FIELD
    
        The constant string field representing a field of buoys.
        Actual string value is ``BuoyField``.

        :type: str

    .. attribute:: FIND_THE_BREAK
    
        The constant string field representing ???.
        Actual string value is ``FindBreak``.

        :type: str

    .. attribute:: CORAL_SURVEY
    
        The constant string field representing ???.
        Actual string value is ``CoralSurvey``.

        :type: str

    .. attribute:: ACOUSTIC_PINGER
    
        The constant string field representing an acoustic pinger.
        Actual string value is ``AcousticPinger``.

        :type: str

    .. attribute:: EMPTY_SPACE
    
        The constant string field representing empty space.
        Actual string value is ``EmptySpace``.

        :type: str

    .. attribute:: position

        Estimated position of the object.

        :type: Point

    .. attribute:: id

        The ID of the object.

        :type: int

    .. attribute:: confidence

        The confidence of the object detection, from 0 to 255.

        :type: int

    .. attribute:: size

        The size of the object in all dimensions.

        :type: Vector3

    .. attribute:: points

        The points (in the ENU frame) that belong to the buoy.

        :type: List[Point32]

    .. attribute:: intensity

        The intensity of each point in the ENU frame.

        :type: List[int]

    .. attribute:: pclInliers

        The inliners for the PCL plane fitting algorithm.

        :type: int

    .. attribute:: normal

        Unit vector for normal to one of the vertical planes of the object.

        :type: Vector3

    .. attribute:: color

        Average color of the buoy.

        :type: ColorRGBA

PerceptionObjectArray
~~~~~~~~~~~~~~~~~~~~~

.. attributetable:: navigator_msgs.msg._PerceptionObjectArray.PerceptionObjectArray

.. class:: navigator_msgs.msg._PerceptionObjectArray.PerceptionObjectArray

    A custom message definition to represent an array of perception objects.

    .. attribute:: objects

        The objects in the array.

        :type: List[PerceptionObject]

Networking Messages
^^^^^^^^^^^^^^^^^^^

Host
~~~~

.. attributetable:: navigator_msgs.msg._Host.Host

.. class:: navigator_msgs.msg._Host.Host

    A custom message definition responsible for associating a hostname and IP
    address with a status.

    .. attribute:: hostname

        The name of the host.

        :type: str

    .. attribute:: ip

        The IP address of the host.

        :type: str

    .. attribute:: status

        The status of the host.

        :type: str

Hosts
~~~~~

.. attributetable:: navigator_msgs.msg._Hosts.Hosts

.. class:: navigator_msgs.msg._Hosts.Hosts

    A custom message definition representing a group of hosts together.

    .. attribute:: hostnames

        A custom, constant string representing a group of hostnames. The string is
        equal to:

            mil-nav-wamv mil-nav-ubnt-wamv mil-nav-ubnt-shore mil-com-velodyne-vlp16 mil-com-sick-lms111

        :type: str

    .. attribute:: hosts
    
        The hosts belonging to the group.

        :type: List[~navigator_msgs.msg._Host.Host]
        
Passive Sonar Messages
^^^^^^^^^^^^^^^^^^^^^^

HydrophoneSamples
~~~~~~~~~~~~~~~~~

.. attributetable:: mil_passive_sonar.msg._HydrophoneSamples.HydrophoneSamples

.. class:: mil_passive_sonar.msg._HydrophoneSamples.HydrophoneSamples
    
    A custom message definition to represent data coming from the hydrophones.

    .. attribute:: channels

        The number of channels supported by the hydrophones.

        :type: int

    .. attribute:: samples

        The number of samples on each channel.

        :type: int

    .. attribute:: sample_rate

        The rate at which samples are recorded, per channel. Equal to the number
        of samples per second.

        :type: int

    .. attribute:: data

        The data recorded from the hydrophones. Each "word" in the data is a piece
        of data from one hydrophone, such as ``H0 H1 H2 H3 H0 H1 ...``.

        :type: List[int]

HydrophoneSamplesStamped
~~~~~~~~~~~~~~~~~~~~~~~~

.. attributetable:: mil_passive_sonar.msg._HydrophoneSamplesStamped.HydrophoneSamplesStamped

.. class:: mil_passive_sonar.msg._HydrophoneSamplesStamped.HydrophoneSamplesStamped
    
    A custom message definition to represent time-stamped data coming from the hydrophones.

    .. attribute:: header

        The message header.

        :type: Header

    .. attribute:: hydrophone_samples

        The hydrophone samples received.

        :type: HydrophoneSamples

Ping
~~~~

.. attributetable:: mil_passive_sonar.msg._Ping.Ping

.. class:: mil_passive_sonar.msg._Ping.Ping
    
    A custom message definition to represent a ping from channels of data.

    .. danger::

        This class is deprecated, and has been replaced by 
        :class:`~mil_passive_sonar.msg._HydrophoneSamples.HydrophoneSamples` and 
        :class:`~mil_passive_sonar.msg._HydrophoneSamplesStamped.HydrophoneSamplesStamped`.
        Support for this message type throughout the repository still exists,
        although it may be removed in the future.

    .. attribute:: header

        The message header.

        :type: Header

    .. attribute:: channels

        The number of channels supported by the hydrophones.

        :type: int

    .. attribute:: samples

        The number of samples on each channel.

        :type: int

    .. attribute:: sample_rate

        The rate at which samples are recorded, per channel. Equal to the number
        of samples per second.

        :type: int

    .. attribute:: data

        The data recorded from the hydrophones. Each "word" in the data is a piece
        of data from one hydrophone, such as ``H0 H1 H2 H3 H0 H1 ...``.

        :type: List[int]

ProcessedPing
~~~~~~~~~~~~~

.. attributetable:: mil_passive_sonar.msg._ProcessedPing.ProcessedPing

.. class:: mil_passive_sonar.msg._ProcessedPing.ProcessedPing
    
    A custom message definition to represent a ping from channels of data.

    .. attribute:: header

        The message header.

        :type: Header

    .. attribute:: position

        The position of the processed ping.

        :type: Point

    .. attribute:: freq

        The frequency of the processed ping.

        :type: float

    .. attribute:: amplitude

        The amplitude of the processed ping.

        :type: float

    .. attribute:: valid

        Whether the processed ping is valid.

        :type: bool

Triggered
~~~~~~~~~

.. attributetable:: mil_passive_sonar.msg._Triggered.Triggered

.. class:: mil_passive_sonar.msg._Triggered.Triggered
    
    A custom message definition to represent ???.

    .. attribute:: header

        The message header.

        :type: Header

    .. attribute:: hydrophone_samples

        ???

        :type: HydrophoneSamples

    .. attribute:: trigger_time

        The time at which the ping was detected.

        :type: float

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

Motor Feedback
~~~~~~~~~~~~~~
.. attributetable:: roboteq_msgs.msg._Feedback.Feedback

.. class:: roboteq_msgs.msg._Feedback.Feedback

    A third-party ROS message type for getting feedback from motor controllers.

    .. attribute:: header

        The header of the message.

        :type: Header

    .. attribute:: motor_current

        Current flowing through the motors.

        :type: float

    .. attribute:: motor_power

        Relative motor power, as a proportion of the full motor power.
        Lives in a range from -1 to 1.

        :type: float

    .. attribute:: commanded_velocity

        The velocity commanded of the motor. Output is in ``rad/s``.

        :type: float

    .. attribute:: measured_velocity

        The true velocity of the motor. Output is in ``rad/s``.

        :type: float

    .. attribute:: measured_position

        The position of the motor, in ``rad``. Wraps around -6/6M.

        :type: float

    .. attribute:: supply_voltage

        The voltage supplied to the motor, in volts.

        :type: float

    .. attribute:: supply_current

        The current supplied to the motor, in amps.

        :type: float

    .. attribute:: motor_temperature

        The temperature of the motor, in Celsius.

        :type: float

    .. attribute:: channel_temperature

        The temperature of the FETs, as reported by the controller. Units are in
        Celsius.

        :type: float

Motor Status
~~~~~~~~~~~~
.. attributetable:: roboteq_msgs.msg._Status.Status

.. class:: roboteq_msgs.msg._Status.Status

    A third-party ROS message type for getting status from motor controllers.

    .. attribute:: header

        The header of the message.

        :type: Header

    .. attribute:: fault

        A representation of any fault that occurred in the motor. Likely one of the
        enumerated fault types of this class.
 
        :type: int

    .. attribute:: FAULT_OVERHEAT

        Constant attribute used to represent that the motor experienced a fault
        as a result of overheating.

        :type: int
        :value: 1

    .. attribute:: FAULT_OVERVOLTAGE

        Constant attribute used to represent that the motor experienced a fault
        as a result of too much voltage.

        :type: int
        :value: 2

    .. attribute:: FAULT_UNDERVOLTAGE

        Constant attribute used to represent that the motor experienced a fault
        as a result of too little voltage.

        :type: int
        :value: 4

    .. attribute:: FAULT_SHORT_CIRCUIT

        Constant attribute used to represent that the motor experienced a fault
        as a result of a short circuit.

        :type: int
        :value: 8

    .. attribute:: FAULT_EMERGENCY_STOP

        Constant attribute used to represent that the motor experienced a fault
        as a result of an emergency stop.

        :type: int
        :value: 16

    .. attribute:: FAULT_SEPEX_EXCITATION_FAULT

        Constant attribute used to represent that the motor experienced a fault
        as a result of an excitation error.

        :type: int
        :value: 32

    .. attribute:: FAULT_MOSFET_FAILURE

        Constant attribute used to represent that the motor experienced a fault
        as a result of a failure in the MOSFET system.

        :type: int
        :value: 64

    .. attribute:: FAULT_STARTUP_CONFIG_FAULT

        Constant attribute used to represent that the motor experienced a fault
        as a result of a failure in the startup configuration.

        :type: int
        :value: 128

    .. attribute:: status

        The status of the motor. Likely set to a combination of the class' enumerated
        types.

        :type: int

    .. attribute:: STATUS_SERIAL_MODE

        Constant attribute used to represent that the motor is in serial mode.

        :type: int
        :value: 1

    .. attribute:: STATUS_PULSE_MODE

        Constant attribute used to represent that the motor is in pulse mode.

        :type: int
        :value: 2

    .. attribute:: STATUS_ANALOG_MODE

        Constant attribute used to represent that the motor is in analog mode.

        :type: int
        :value: 4

    .. attribute:: STATUS_POWER_STAGE_OFF

        Constant attribute used to represent that the power stage of the motor is
        off.

        :type: int
        :value: 8

    .. attribute:: STATUS_STALL_DETECTED

        Constant attribute used to represent that a stall was detected.

        :type: int
        :value: 16

    .. attribute:: STATUS_AT_LIMIT

        Constant attribute used to represent that the motor is at its limit.

        :type: int
        :value: 32

    .. attribute:: STATUS_MICROBASIC_SCRIPT_RUNNING

        Constant attribute used to represent that the microbasic script is running.

        :type: int
        :value: 128

    .. attribute:: ic_temperature

        The temperature of the main logic chip, in Celsius.

        :type: float

    .. attribute:: internal_voltage

        The internal voltage, in volts.

        :type: float

    .. attribute:: adc_voltage

        The voltage of the analog-to-digital converter, in volts.

        :type: float

VRX Messages
^^^^^^^^^^^^

DockShape
~~~~~~~~~

.. attributetable:: navigator_msgs.msg._DockShape.DockShape

.. class:: navigator_msgs.msg._DockShape.DockShape

    .. attribute:: Shape

        The shape of the dock. Likely either :attr:`DockShape.CROSS`, :attr:`DockShape.CIRCLE`,
        or :attr:`DockShape.TRIANGLE`.

        :type: str

    .. attribute:: CROSS

        The dock holds the cross shape.

        :type: str

    .. attribute:: CIRCLE

        The dock holds the circle shape.

        :type: str

    .. attribute:: TRIANGLE

        The dock holds the triangle shape.

        :type: str

    .. attribute:: Color

        The color of the dock. Likely either :attr:`DockShape.RED`, :attr:`DockShape.BLUE`,
        or :attr:`DockShape.GREEN`.

        :type: str

    .. attribute:: RED

        The dock shape is red.

        :type: str

    .. attribute:: BLUE

        The dock shape is blue.

        :type: str

    .. attribute:: GREEN

        The dock shape is green.

        :type: str

    .. attribute:: CenterX

        The x-dimension of the center of the dock shape.

        :type: int

    .. attribute:: CenterY

        The y-dimension of the center of the dock shape.

        :type: int

    .. attribute.. img_width

        The width of the image showing the dock and its shape.        

        :type: int

    .. attribute:: header

        The message header.

        :type: Header

    .. attribute:: points

        ???

        :type: List[~geometry_msgs.msg._Point.Point]

    .. attribute:: color_confidence

        The relative confidence that the color prediction is correct.

        :type: float

    .. attribute:: shape_confidence

        The relative confidence that the shape prediction is correct.

        :type: float

DockShapes
~~~~~~~~~~

.. attributetable:: navigator_msgs.msg._DockShapes.DockShapes

.. class:: navigator_msgs.msg._DockShapes.DockShapes

    A custom message definition to represent the presence of multiple dock shapes
    found by the vision system.

    .. attribute:: list

        The list of shapes found.

        :type: List[DockShape]


ScanTheCode
~~~~~~~~~~~

.. attributetable:: navigator_msgs.msg._ScanTheCode.ScanTheCode

.. class:: navigator_msgs.msg._ScanTheCode.ScanTheCode

    A custom message definition to represent the color pattern show by a Scan The
    Code totem.

    .. attribute:: string_pattern

        The pattern shown. ``R`` stands for red, ``B`` stands for blue, and ``G``
        stands for green.

        :type: str

Services
--------

Mission Systems
^^^^^^^^^^^^^^^

AcousticBeacon
~~~~~~~~~~~~~~

.. attributetable:: navigator_msgs.srv._AcousticBeacon.AcousticBeaconRequest

.. class:: navigator_msgs.srv._AcousticBeacon.AcousticBeaconRequest

   The request class for the ``navigator_msgs/AcousticBeacon`` service. The class
   contains no settable attributes.

.. attributetable:: navigator_msgs.srv._AcousticBeacon.AcousticBeaconResponse

.. class:: navigator_msgs.srv._AcousticBeacon.AcousticBeaconResponse

   The repsonse class for the ``navigator_msgs/AcousticBeacon`` service.

   .. attribute:: beacon_position

        The position of the acoustic beacon.

        :type: Point

   .. attribute:: setValue

        Whether the position data of the beacon is reliable enough to be used.

        :type: bool

ChooseAnimal
~~~~~~~~~~~~
.. attributetable:: navigator_msgs.srv._ChooseAnimal.ChooseAnimalRequest

.. class:: navigator_msgs.srv._ChooseAnimal.ChooseAnimalRequest

   The request class for the ``navigator_msgs/ChooseAnimal`` service.

   .. attribute:: target_animal

        The target animal to circle around. Should be ``platyus``, ``crocodile``,
        or ``turtle``.

        :type: str

   .. attribute:: circle_direction

        The direction to circle in. Should be ``clockwise`` or ``anti-clockwise``.

        :type: str

.. attributetable:: navigator_msgs.srv._ChooseAnimal.ChooseAnimalResponse

.. class:: navigator_msgs.srv._ChooseAnimal.ChooseAnimalResponse

   The repsonse class for the ``navigator_msgs/ChooseAnimal`` service.

   .. attribute:: movement_complete

        Whether the movement was completed.

        :type: bool

ColorRequest
~~~~~~~~~~~~
.. attributetable:: navigator_msgs.srv._ColorRequest.ColorRequestRequest

.. class:: navigator_msgs.srv._ColorRequest.ColorRequestRequest

   The request class for the ``navigator_msgs/ColorRequest`` service.

   .. attribute:: color

        The color used to find objects with.

        :type: str

.. attributetable:: navigator_msgs.srv._ColorRequest.ColorRequestResponse

.. class:: navigator_msgs.srv._ColorRequest.ColorRequestResponse

   The repsonse class for the ``navigator_msgs/ColorRequest`` service.

   .. attribute:: found
    
        Whether objects were found.

        :type: bool

   .. attribute:: ids
    
        The IDs of objects that were found.

        :type: List[int]

FindPinger
~~~~~~~~~~
.. attributetable:: navigator_msgs.srv._FindPinger.FindPingerRequest

.. class:: navigator_msgs.srv._FindPinger.FindPingerRequest

   The request class for the ``navigator_msgs/FindPinger`` service. The request
   has no individual attributes.

.. attributetable:: navigator_msgs.srv._FindPinger.FindPingerResponse

.. class:: navigator_msgs.srv._FindPinger.FindPingerResponse

   The repsonse class for the ``navigator_msgs/FindPinger`` service.

   .. attribute:: pinger_position
    
        The position of the pinger.

        :type: Point

   .. attribute:: num_samples
    
        ???

        :type: int

GetDockBays
~~~~~~~~~~~
.. attributetable:: navigator_msgs.srv._GetDockBays.GetDockBaysRequest

.. class:: navigator_msgs.srv._GetDockBays.GetDockBaysRequest

   The request class for the ``navigator_msgs/GetDockBays`` service. The request
   has no individual attributes.

.. attributetable:: navigator_msgs.srv._GetDockBays.GetDockBaysResponse

.. class:: navigator_msgs.srv._GetDockBays.GetDockBaysResponse

   The repsonse class for the ``navigator_msgs/GetDockBays`` service.

   .. attribute:: bays
    
        The positions of the three dock bays in the ENU frame. The first element is
        the position of the left dock, the second element is the position of the center dock,
        the third element is the position of the right dock.

        :type: List[Point]

   .. attribute:: normal
    
        The normal vector pointing away from the plane of dock back.

        :type: Vector3

   .. attribute:: success
    
        Whether the position of the docks could be found.

        :type: bool

   .. attribute:: error
    
        If :attr:`~navigator_msgs.srv._GetDockBays.GetDockBays.success` is ``False``,
        then a message describing what went wrong.

        :type: str

GetDockShape
~~~~~~~~~~~~
.. attributetable:: navigator_msgs.srv._GetDockShape.GetDockShapeRequest

.. class:: navigator_msgs.srv._GetDockShape.GetDockShapeRequest

   The request class for the ``navigator_msgs/GetDockShape`` service.

   .. attribute:: Shape

        The shape to the get the associated dock of. Likely one of the associated shape
        enumerations.

        :type: str

   .. attribute:: CROSS

        Constant string attribute used to represent a cross shape on a dock. True value
        is set to ``CROSS``.

        Likely used in the :attr:`~navigator_msgs.srv._GetDockShape.GetDockShapeRequest.Shape` attribute.

        :type: str

   .. attribute:: TRIANGLE

        Constant string attribute used to represent a triangle shape on a dock. True value
        is set to ``TRIANGLE``.

        Likely used in the :attr:`~navigator_msgs.srv._GetDockShape.GetDockShapeRequest.Shape` attribute.

        :type: str

   .. attribute:: CIRCLE

        Constant string attribute used to represent a circle shape on a dock. True value
        is set to ``CIRCLE``.

        Likely used in the :attr:`~navigator_msgs.srv._GetDockShape.GetDockShapeRequest.Shape` attribute.

        :type: str

   .. attribute:: Color

        The color to the get the associated dock of. Likely one of the associated color
        enumerations.

        :type: str

   .. attribute:: RED

        Constant string attribute used to represent a red shape on a dock. True value
        is set to ``RED``.

        Likely used in the :attr:`~navigator_msgs.srv._GetDockShape.GetDockShapeRequest.Color` attribute.

        :type: str

   .. attribute:: BLUE

        Constant string attribute used to represent a triangle shape on a dock. True value
        is set to ``BLUE``.

        Likely used in the :attr:`~navigator_msgs.srv._GetDockShape.GetDockShapeRequest.Color` attribute.

        :type: str

   .. attribute:: GREEN

        Constant string attribute used to represent a circle shape on a dock. True value
        is set to ``GREEN``.

        Likely used in the :attr:`~navigator_msgs.srv._GetDockShape.GetDockShapeRequest.Color` attribute.

        :type: str

   .. attribute:: ANY

        Constant string attribute used to represent any value for a specific field - ie, a
        dock with any shape or color representation. Actual value is ``ANY``.

        :type: str

.. attributetable:: navigator_msgs.srv._GetDockShape.GetDockShapeResponse

.. class:: navigator_msgs.srv._GetDockShape.GetDockShapeResponse

   The repsonse class for the ``navigator_msgs/GetDockShape`` service.

   .. attribute:: symbol

        The associated shape and color of the returned dock.

        :type: DockShape

   .. attribute:: found
    
        Whether a viable dock was found.

        :type: bool

   .. attribute:: error
    
        If :attr:`~navigator_msgs.srv._GetDockShape.GetDockShapeResponse.found` was false,
        then a description of what went wrong. May be equal to one of this class' enumerations.

        :type: str

   .. attribute:: INVALID_REQUEST

        An enumeration to describe a request that was invalid in some way. Actual
        string value is ``INVALID_REQUEST``.

        :type: str

   .. attribute:: NODE_DISABLED

        An enumeration to describe a request that was invalid in some way. Actual
        string value is ``NODE_DISABLED``.

        :type: str

   .. attribute:: TOO_SMALL_SAMPLE

        An enumeration to describe a request that was invalid in some way. Actual
        string value is ``TOO_SMALL_SAMPLE``.

        :type: str

   .. attribute:: SHAPE_NOT_FOUND

        An enumeration to describe a request that was invalid in some way. Actual
        string value is ``SHAPE_NOT_FOUND``.

        :type: str

GetDockShapes
~~~~~~~~~~~~~
.. attributetable:: navigator_msgs.srv._GetDockShapes.GetDockShapesRequest

.. class:: navigator_msgs.srv._GetDockShapes.GetDockShapesRequest

   The request class for the ``navigator_msgs/GetDockShapes`` service.

   .. attribute:: Shape

        The shape to the get the associated dock of. Likely one of the associated shape
        enumerations.

        :type: str

   .. attribute:: CROSS

        Constant string attribute used to represent a cross shape on a dock. True value
        is set to ``CROSS``.

        Likely used in the :attr:`~navigator_msgs.srv._GetDockShape.GetDockShapeRequest.Shape` attribute.

        :type: str

   .. attribute:: TRIANGLE

        Constant string attribute used to represent a triangle shape on a dock. True value
        is set to ``TRIANGLE``.

        Likely used in the :attr:`~navigator_msgs.srv._GetDockShape.GetDockShapeRequest.Shape` attribute.

        :type: str

   .. attribute:: CIRCLE

        Constant string attribute used to represent a circle shape on a dock. True value
        is set to ``CIRCLE``.

        Likely used in the :attr:`~navigator_msgs.srv._GetDockShape.GetDockShapeRequest.Shape` attribute.

        :type: str

   .. attribute:: Color

        The color to the get the associated dock of. Likely one of the associated color
        enumerations.

        :type: str

   .. attribute:: RED

        Constant string attribute used to represent a red shape on a dock. True value
        is set to ``RED``.

        Likely used in the :attr:`~navigator_msgs.srv._GetDockShape.GetDockShapeRequest.Color` attribute.

        :type: str

   .. attribute:: BLUE

        Constant string attribute used to represent a triangle shape on a dock. True value
        is set to ``BLUE``.

        Likely used in the :attr:`~navigator_msgs.srv._GetDockShape.GetDockShapeRequest.Color` attribute.

        :type: str

   .. attribute:: GREEN

        Constant string attribute used to represent a circle shape on a dock. True value
        is set to ``GREEN``.

        Likely used in the :attr:`~navigator_msgs.srv._GetDockShape.GetDockShapeRequest.Color` attribute.

        :type: str

   .. attribute:: ANY

        Constant string attribute used to represent any value for a specific field - ie, a
        dock with any shape or color representation. Actual value is ``ANY``.

        :type: str

.. attributetable:: navigator_msgs.srv._GetDockShapes.GetDockShapesResponse

.. class:: navigator_msgs.srv._GetDockShapes.GetDockShapesResponse

   The repsonse class for the ``navigator_msgs/GetDockShapes`` service.

   .. attribute:: shapes

        The relevant dock shapes that were found.

        :type: List[DockShape]

   .. attribute:: found
    
        If one or more suitable shapes was returned, then true.

        :type: bool

   .. attribute:: error
    
        If :attr:`~navigator_msgs.srv._GetDockShapes.GetDockShapesResponse.found`
        was false, then an explanation of why.

        :type: str

   .. attribute:: INVALID_REQUEST

        An enumeration to describe a request that was invalid in some way. Actual
        string value is ``INVALID_REQUEST``.

        :type: str

   .. attribute:: NODE_DISABLED

        An enumeration to describe a request that was invalid in some way. Actual
        string value is ``NODE_DISABLED``.

        :type: str

   .. attribute:: TOO_SMALL_SAMPLE

        An enumeration to describe a request that was invalid in some way. Actual
        string value is ``TOO_SMALL_SAMPLE``.

        :type: str

   .. attribute:: SHAPE_NOT_FOUND

        An enumeration to describe a request that was invalid in some way. Actual
        string value is ``SHAPE_NOT_FOUND``.

        :type: str

MessageDetectDeliver
~~~~~~~~~~~~~~~~~~~~
.. attributetable:: navigator_msgs.srv._MessageDetectDeliver.MessageDetectDeliverRequest

.. class:: navigator_msgs.srv._MessageDetectDeliver.MessageDetectDeliverRequest

   The request class for the ``navigator_msgs/MessageDetectDeliver`` service.

   .. attribute:: shape_color

        The color of the requested shape.

        :type: str

   .. attribute:: shape

        The type of requested shape.

        :type: str

.. attributetable:: navigator_msgs.srv._MessageDetectDeliver.MessageDetectDeliverResponse

.. class:: navigator_msgs.srv._MessageDetectDeliver.MessageDetectDeliverResponse

   The repsonse class for the ``navigator_msgs/MessageDetectDeliver`` service.

   .. attribute:: message

        A message in response to the process.

        :type: str

MessageExtranceExitGate
~~~~~~~~~~~~~~~~~~~~~~~
.. attributetable:: navigator_msgs.srv._MessageExtranceExitGate.MessageExtranceExitGateRequest

.. class:: navigator_msgs.srv._MessageExtranceExitGate.MessageExtranceExitGateRequest

   The request class for the ``navigator_msgs/MessageExtranceExitGate`` service.

   .. attribute:: entrance_gate

        The entrance gate relevant to the task.

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

.. attributetable:: navigator_msgs.srv._MessageExtranceExitGate.MessageExtranceExitGateResponse

.. class:: navigator_msgs.srv._MessageExtranceExitGate.MessageExtranceExitGateResponse

   The repsonse class for the ``navigator_msgs/MessageExtranceExitGate`` service.

   .. attribute:: message

        A message in response to the process.

        :type: str

ScanTheCodeMission
~~~~~~~~~~~~~~~~~~
.. attributetable:: navigator_msgs.srv._ScanTheCodeMission.ScanTheCodeMissionRequest

.. class:: navigator_msgs.srv._ScanTheCodeMission.ScanTheCodeMissionRequest

   The request class for the ``navigator_msgs/ScanTheCodeMission`` service.

   .. attribute:: object

        The perception object to look for.

        :type: PerceptionObject

.. attributetable:: navigator_msgs.srv._ScanTheCodeMission.ScanTheCodeMissionResponse

.. class:: navigator_msgs.srv._ScanTheCodeMission.ScanTheCodeMissionResponse

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

ShooterManual
~~~~~~~~~~~~~
.. attributetable:: navigator_msgs.srv._ShooterManual.ShooterManualRequest

.. class:: navigator_msgs.srv._ShooterManual.ShooterManualRequest

   The request class for the ``navigator_msgs/ShooterManual`` service.

   .. attribute:: feeder

        ???

        :type: float

   .. attribute:: shooter

        ???

        :type: float

.. attributetable:: navigator_msgs.srv._ShooterManual.ShooterManualResponse

.. class:: navigator_msgs.srv._ShooterManual.ShooterManualResponse

   The repsonse class for the ``navigator_msgs/ShooterManual`` service.

   .. attribute:: success

        Whether the shooter operation was successful.

        :type: bool

StartGate
~~~~~~~~~
.. attributetable:: navigator_msgs.srv._StartGate.StartGateRequest

.. class:: navigator_msgs.srv._StartGate.StartGateRequest

   The request class for the ``navigator_msgs/StartGate`` service. The request
   class no public attributes.

.. attributetable:: navigator_msgs.srv._StartGate.StartGateResponse

.. class:: navigator_msgs.srv._StartGate.StartGateResponse

   The repsonse class for the ``navigator_msgs/StartGate`` service.

   .. attribute:: target

        The target of the mission's start gate.

        :type: PoseStamped

   .. attribute:: success

        Whether the start gate operation was successful.

        :type: bool

Subsystems
^^^^^^^^^^

AlarmGet
~~~~~~~~

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
~~~~~~~~

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

CameraDBQuery
~~~~~~~~~~~~~
.. attributetable:: navigator_msgs.srv._CameraDBQuery.CameraDBQueryRequest

.. class:: navigator_msgs.srv._CameraDBQuery.CameraDBQueryRequest

   The request class for the ``navigator_msgs/CameraDBQuery`` service.

   .. attribute:: name

        The name of the object to query.

        :type: str

   .. attribute:: id

        The ID of the object to query.

        :type: int

.. attributetable:: navigator_msgs.srv._CameraDBQuery.CameraDBQueryResponse

.. class:: navigator_msgs.srv._CameraDBQuery.CameraDBQueryResponse

   The repsonse class for the ``navigator_msgs/CameraDBQuery`` service.

   .. attribute:: found

        Whether the object is found.

        :type: bool

MoveToWaypoint
~~~~~~~~~~~~~~
.. attributetable:: navigator_msgs.srv._MoveToWaypoint.MoveToWaypointRequest

.. class:: navigator_msgs.srv._MoveToWaypoint.MoveToWaypointRequest

   The request class for the ``navigator_msgs/MoveToWaypoint`` service.

   .. attribute:: target_p

        The target pose to head toward.

        :type: Pose

.. attributetable:: navigator_msgs.srv._MoveToWaypoint.MoveToWaypointResponse

.. class:: navigator_msgs.srv._MoveToWaypoint.MoveToWaypointResponse

   The repsonse class for the ``navigator_msgs/MoveToWaypoint`` service.

   .. attribute:: success

        Whether the movement was successful.

        :type: bool

ObjectDBQuery
~~~~~~~~~~~~~
.. attributetable:: navigator_msgs.srv._ObjectDBQuery.ObjectDBQueryRequest

.. class:: navigator_msgs.srv._ObjectDBQuery.ObjectDBQueryRequest

   The request class for the ``navigator_msgs/ObjectDBQuery`` service.

   .. attribute:: name

        The name of the object to find in the database.

        :type: str

   .. attribute:: cmd

        The command to run in the database. The command should be formatted as
        ``ID=YYY``, where ``ID`` is the property ID of the object to change, and
        ``YYY`` is the value to set.

        :type: str

.. attributetable:: navigator_msgs.srv._ObjectDBQuery.ObjectDBQueryResponse

.. class:: navigator_msgs.srv._ObjectDBQuery.ObjectDBQueryResponse

   The repsonse class for the ``navigator_msgs/ObjectDBQuery`` service.

   .. attribute:: found

        Whether the requested object was found.

        :type: bool

   .. attribute:: objects

        A list of all objects found.

        :type: List[PerceptionObject]

SetFrequency
~~~~~~~~~~~~
.. attributetable:: navigator_msgs.srv._SetFrequency.SetFrequencyRequest

.. class:: navigator_msgs.srv._SetFrequency.SetFrequencyRequest

   The request class for the ``navigator_msgs/SetFrequency`` service.

   .. attribute:: frequency

        The frequency to set.

        :type: int

.. attributetable:: navigator_msgs.srv._SetFrequency.SetFrequencyResponse

.. class:: navigator_msgs.srv._SetFrequency.SetFrequencyResponse

   The repsonse class for the ``navigator_msgs/SetFrequency`` service. The
   class no public attributes.

SetROI
~~~~~~
.. attributetable:: navigator_msgs.srv._SetROI.SetROIRequest

.. class:: navigator_msgs.srv._SetROI.SetROIRequest

   The request class for the ``navigator_msgs/SetROI`` service.

   .. attribute:: roi

        The region of interest to set.

        :type: RegionOfInterest

.. attributetable:: navigator_msgs.srv._SetROI.SetROIResponse

.. class:: navigator_msgs.srv._SetROI.SetROIResponse

   The repsonse class for the ``navigator_msgs/SetROI`` service.

   .. attribute:: success

        Whether the set operation was successful.

        :type: bool

   .. attribute:: error

        If the operation failed, then a description of what went wrong.

        :type: str

   .. attribute:: OUTSIDE_OF_FRAME

        A string constant to represent that the region of interest is outside the
        observable frame. Constant string actually equally to ``OUTSIDE_OF_FRAME``.

        :type: str

StereoShapeDetector
~~~~~~~~~~~~~~~~~~~
.. attributetable:: navigator_msgs.srv._StereoShapeDetector.StereoShapeDetectorRequest

.. class:: navigator_msgs.srv._StereoShapeDetector.StereoShapeDetectorRequest

   The request class for the ``navigator_msgs/StereoShapeDetector`` service.

   .. attribute:: detection_switch

        ???

        :type: bool
        
   .. attribute:: shape

        ???

        :type: str

   .. attribute:: processing_type

        ???

        :type: str

   .. attribute:: num_points

        The number of points relevant to the detector.

        :type: int

   .. attribute:: model_params

        ???

        :type: List[float]

.. attributetable:: navigator_msgs.srv._StereoShapeDetector.StereoShapeDetectorResponse

.. class:: navigator_msgs.srv._StereoShapeDetector.StereoShapeDetectorResponse

   The repsonse class for the ``navigator_msgs/StereoShapeDetector`` service.

   .. attribute:: success

        Whether the detector was succesful in detecting!

        :type: bool

VisionRequest
~~~~~~~~~~~~~
.. attributetable:: navigator_msgs.srv._VisionRequest.VisionRequestRequest

.. class:: navigator_msgs.srv._VisionRequest.VisionRequestRequest

   The request class for the ``navigator_msgs/VisionRequest`` service.

   .. attribute:: target_name

        The target to look for in the vision system.

        :type: str
        
.. attributetable:: navigator_msgs.srv._VisionRequest.VisionRequestResponse

.. class:: navigator_msgs.srv._VisionRequest.VisionRequestResponse

   The repsonse class for the ``navigator_msgs/VisionRequest`` service.

   .. attribute:: pose

        Where the object is at, in the vision system.

        :type: PoseStamped

   .. attribute:: covariance_diagonal

        The covariance in the vision target.

        :type: Vector3

   .. attribute:: found
   
        Whether the vision object was found.

        :type: bool

Standard Messages
^^^^^^^^^^^^^^^^^

SetBool
~~~~~~~

.. attributetable:: std_srvs.srv._SetBool.SetBoolRequest

.. class:: std_srvs.srv._SetBool.SetBoolRequest

    The request type for the ``SetBool`` service. Requests for some boolean value
    to be set.

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

Conversions
^^^^^^^^^^^

CameraToLidarTransform
~~~~~~~~~~~~~~~~~~~~~~
.. attributetable:: navigator_msgs.srv._CameraToLidarTransform.CameraToLidarTransformRequest

.. class:: navigator_msgs.srv._CameraToLidarTransform.CameraToLidarTransformRequest

   The request class for the ``navigator_msgs/CameraToLidarTransform`` service.

   .. attribute:: header

        The stamp the point was seen for tf.

        :type: Header

   .. attribute:: point

        The x-dimension and y-dimension of the point in the camera. The z-dimension
        is ignored.

        :type: Point

   .. attribute:: tolerance

        The number of pixels the projected 3D Lidar point can be from the target point
        to be included in the response.

        :type: int

.. attributetable:: navigator_msgs.srv._CameraToLidarTransform.CameraToLidarTransformResponse

.. class:: navigator_msgs.srv._CameraToLidarTransform.CameraToLidarTransformResponse

   The repsonse class for the ``navigator_msgs/CameraToLidarTransform`` service.

   .. attribute:: success

        True if at least one point is found within LIDAR and transformed.

        :type: bool

   .. attribute:: transformed

        If success is true, then the list of transformed points.

        :type: List[Point]

   .. attribute:: closest

        3D point that is closest to the target point when transformed and projected

        :type: Point

   .. attribute:: normal

        The normal unit vector in the camera frame estimated from the transformed points.

        :type: Vector3

   .. attribute:: distance

        The mean z-dimension of the transformed points.

        :type: float

   .. attribute:: error

        If success is false, then what went wrong.

        :type: str

   .. attribute:: CLOUD_NOT_FOUND

        The pointcloud was not found. Constant string actually equal to ``pointcloud
        not found``.

        :type: str

   .. attribute:: NO_POINTS

        No points were found. Constant string actually equal to ``no points``.

        :type: str

CoordinateConversion
~~~~~~~~~~~~~~~~~~~~

.. attributetable:: navigator_msgs.srv._CoordinateConversion.CoordinateConversionRequest

.. class:: navigator_msgs.srv._CoordinateConversion.CoordinateConversionRequest

   The request class for the ``navigator_msgs/CoordinateConversion`` service.

   .. attribute:: LLA

        The longitude, latitude, altitude coordinate frame. Constant string equal
        to ``lla``.

        :type: str

   .. attribute:: ENU

        The east, north, up frame. Constant string equal to ``enu``.

        :type: str

   .. attribute:: ECEF

        The Earth-centered, Earth-fixed frame. Constant string equal to ``ecef``.

        :type: str

   .. attribute:: frame

        The current frame of the relative objects.

        :type: str

   .. attribute:: to_frame

        The frame of objects to convert objects to.

        :type: str

   .. attribute:: points

        The points to convert between the different frames.

        :type: List[Point]

.. attributetable:: navigator_msgs.srv._CoordinateConversion.CoordinateConversionResponse

.. class:: navigator_msgs.srv._CoordinateConversion.CoordinateConversionResponse

   The repsonse class for the ``navigator_msgs/CoordinateConversion`` service.

   .. attribute:: converted
    
        The list of converted points.

        :type: List[Point]

   .. attribute:: message
    
        If an error occurred, the message of what went wrong.

        :type: str

KeyboardControl
^^^^^^^^^^^^^^^
.. attributetable:: navigator_msgs.srv._KeyboardControl.KeyboardControlRequest

.. class:: navigator_msgs.srv._KeyboardControl.KeyboardControlRequest

   The request class for the ``navigator_msgs/KeyboardControl`` service.

   .. attribute:: uuid

        A unique ID to represent the process (?).

        :type: str

   .. attribute:: keycode

        The keycode that was pressed.

        :type: int

.. attributetable:: navigator_msgs.srv._KeyboardControl.KeyboardControlResponse

.. class:: navigator_msgs.srv._KeyboardControl.KeyboardControlResponse

   The repsonse class for the ``navigator_msgs/KeyboardControl`` service.

   .. attribute:: generated_uuid

        A response unique ID that was generated in response to the request.

        :type: str

   .. attribute:: is_locked
    
        Whether the client which sent the keycode has "locked control" of the keyboard
        server, and is therefore blocking other keyboard input.

        :type: bool

Actions
-------

Path Planner
^^^^^^^^^^^^

MoveAction
~~~~~~~~~~

.. attributetable:: navigator_path_planner.msg._MoveAction.MoveAction

.. class:: navigator_path_planner.msg._MoveAction.MoveAction

    A custom message representing the general movement of an entire system.

    .. attribute:: action_goal
    
        The goal for an action's movement.

        :type: MoveActionGoal

    .. attribute:: action_result
    
        The result of a move action's result.

        :type: MoveActionResult

    .. attribute:: action_feedback
    
        The feedback for an action movement.

        :type: MoveActionFeedback

MoveActionResult
~~~~~~~~~~~~~~~~
.. attributetable:: navigator_path_planner.msg._MoveActionResult.MoveActionResult

.. class:: navigator_path_planner.msg._MoveActionResult.MoveActionResult

    A custom message representing the result of a system's movement.

    .. attribute:: header
    
        The header for the message.

        :type: Header

    .. attribute:: status
    
        The status of the system in its movement.

        :type: GoalStatus

    .. attribute:: result
    
        The result of the movement

        :type: MoveResult

MoveActionFeedback
~~~~~~~~~~~~~~~~~~
.. attributetable:: navigator_path_planner.msg._MoveActionFeedback.MoveActionFeedback

.. class:: navigator_path_planner.msg._MoveActionFeedback.MoveActionFeedback

    A custom message representing the feedback of a system's movement.

    .. attribute:: header
    
        The header for the message.

        :type: Header

    .. attribute:: status
    
        The status of the system in its movement.

        :type: GoalStatus

    .. attribute:: feedback
    
        The feedback of the movement.

        :type: MoveFeedback

MoveActionGoal
~~~~~~~~~~~~~~
.. attributetable:: navigator_path_planner.msg._MoveActionGoal.MoveActionGoal

.. class:: navigator_path_planner.msg._MoveActionGoal.MoveActionGoal

    A custom message representing the goal of an object's action movement.

    .. attribute:: header
    
        The header for the message.

        :type: Header

    .. attribute:: goal_id
    
        The ID of the goal.

        :type: GoalID

    .. attribute:: goal
    
        The goal to move to.

        :type: MoveGoal

MoveFeedback
~~~~~~~~~~~~~~~~~~
.. attributetable:: navigator_path_planner.msg._MoveFeedback.MoveFeedback

.. class:: navigator_path_planner.msg._MoveFeedback.MoveFeedback

    A custom message representing the feedback of a system's movement.

    .. attribute:: behavior

        A description of the behavior.

        :type: str

    .. attribute:: tree_size
    
        The size of the lqRRT tree.

        :type: int

    .. attribute:: tracking
    
        ???

        :type: bool

    .. attribute:: distance
    
        ???

        :type: List[float]

    .. attribute:: time_till_next_branch
    
        ???

        :type: float

MoveGoal
~~~~~~~~
.. attributetable:: navigator_path_planner.msg._MoveGoal.MoveGoal

.. class:: navigator_path_planner.msg._MoveGoal.MoveGoal

    A custom message representing the goal of an object's movement.

    .. attribute:: HOLD

        A constant string representing to hold the object's movement. Actually
        equally to ``hold``.

        :type: str

    .. attribute:: DRIVE

        A constant string representing to using driving movement. Actually
        equally to ``drive``.

        :type: str

    .. attribute:: DRIVE_SMOOTH

        A constant string representing to using a smooth driving movement. Actually
        equally to ``drive!``.

        :type: str

    .. attribute:: SKID

        A constant string representing a skidding movement. Actually
        equally to ``skid``.

        :type: str

    .. attribute:: SPIRAL

        A constant string representing a spiral movement. Actually
        equally to ``spiral``.

        :type: str

    .. attribute:: BYPASS

        A constant string representing a spiral movement. Actually
        equally to ``bypass``.

        :type: str

    .. attribute:: move_type

        The type of movement desired, often one of the values above.

        :type: str

    .. attribute:: goal

        The goal to move to.

        :type: Pose

    .. attribute:: focus

        The focal point.

        :type: Point

    .. attribute:: initial_plan_time

        The initial time at which the movement was planned.

        :type: float

    .. attribute:: blind

        ???

        :type: bool

    .. attribute:: speed_factor

        ???

        :type: List[float]

MoveResult
~~~~~~~~~~
.. attributetable:: navigator_path_planner.msg._MoveResult.MoveResult

.. class:: navigator_path_planner.msg._MoveResult.MoveResult

    A custom message representing the goal of an object's movement.

    .. attribute:: failure_reason

        The reason for the movement failing, if any.

        :type: str

Shooter
^^^^^^^

ShooterDoAction
~~~~~~~~~~~~~~~

.. attributetable:: navigator_msgs.msg._ShooterDoAction.ShooterDoAction

.. class:: navigator_msgs.msg._ShooterDoAction.ShooterDoAction

    A custom message representing the general movement of an entire system.

    .. attribute:: action_goal
    
        The goal for an action's movement.

        :type: ShooterDoActionGoal

    .. attribute:: action_result
    
        The result of a move action's result.

        :type: ShooterDoActionResult

    .. attribute:: action_feedback
    
        The feedback for an action movement.

        :type: ShooterDoActionFeedback

ShooterDoActionResult
~~~~~~~~~~~~~~~~~~~~~

.. attributetable:: navigator_msgs.msg._ShooterDoActionResult.ShooterDoActionResult

.. class:: navigator_msgs.msg._ShooterDoActionResult.ShooterDoActionResult

    A custom message representing the result of a system's movement.

    .. attribute:: header
    
        The header for the message.

        :type: Header

    .. attribute:: status
    
        The status of the system in its movement.

        :type: GoalStatus

    .. attribute:: result
    
        The result of the movement

        :type: ShooterDoResult

ShooterDoActionFeedback
~~~~~~~~~~~~~~~~~~~~~~~

.. attributetable:: navigator_msgs.msg._ShooterDoActionFeedback.ShooterDoActionFeedback

.. class:: navigator_msgs.msg._ShooterDoActionFeedback.ShooterDoActionFeedback

    A custom message representing the feedback of a system's movement.

    .. attribute:: header
    
        The header for the message.

        :type: Header

    .. attribute:: status
    
        The status of the system in its movement.

        :type: GoalStatus

    .. attribute:: feedback
    
        The feedback of the movement.

        :type: ShooterDoFeedback

ShooterDoActionGoal
~~~~~~~~~~~~~~~~~~~

.. attributetable:: navigator_msgs.msg._ShooterDoActionGoal.ShooterDoActionGoal

.. class:: navigator_msgs.msg._ShooterDoActionGoal.ShooterDoActionGoal

    A custom message representing the goal of an object's action movement.

    .. attribute:: header
    
        The header for the message.

        :type: Header

    .. attribute:: goal_id
    
        The ID of the goal.

        :type: GoalID

    .. attribute:: goal
    
        The goal to move to.

        :type: ShooterDoGoal

ShooterDoFeedback
~~~~~~~~~~~~~~~~~
.. attributetable:: navigator_msgs.msg._ShooterDoFeedback.ShooterDoFeedback

.. class:: navigator_msgs.msg._ShooterDoFeedback.ShooterDoFeedback

    A custom message representing the feedback of a system's movement.

    .. attribute:: time_remaining

        The amount of time remaining.

        :type: rospy.Duration

ShooterDoGoal
~~~~~~~~~~~~~
.. attributetable:: navigator_msgs.msg._ShooterDoGoal.ShooterDoGoal

.. class:: navigator_msgs.msg._ShooterDoGoal.ShooterDoGoal

    A custom message representing the goal of an object's ShooterDoment. The class
    has no public attributes.

ShooterDoResult
~~~~~~~~~~~~~~~
.. attributetable:: navigator_msgs.msg._ShooterDoResult.ShooterDoResult

.. class:: navigator_msgs.msg._ShooterDoResult.ShooterDoResult

    A custom message representing the goal of an object's ShooterDoment.

    .. attribute:: ALREADY_RUNNING

        A constant string value to enumerate an error that the shooter is already running.
        Constant value set to the name of the variable.

        :type: str

    .. attribute:: NOT_LOADED

        A constant string value to enumerate an error that the shooter is not loaded.
        Constant value set to the name of the variable.

        :type: str

    .. attribute:: ALREADY_LOADAED

        A constant string value to enumerate an error that the shooter is already loaded.
        Constant value set to the name of the variable.

        :type: str

    .. attribute:: MANUAL_CONTROL_USED

        A constant string value to enumerate an error that the shooter is being controlled manually.
        Constant value set to the name of the variable.

        :type: str

    .. attribute:: KILLED

        A constant string value to enumerate an error that the shooter process was killed.
        Constant value set to the name of the variable.

        :type: str

    .. attribute:: success

        A constant string value to enumerate an error that the shooter process was successful.

        :type: bool

    .. attribute:: error

        If success was not had, then what went wrong.

        :type: str
        
Exceptons
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

FPrintFactory
^^^^^^^^^^^^^

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

BagCrawler
^^^^^^^^^^

.. attributetable:: BagCrawler

.. autoclass:: BagCrawler
    :members:

CvDebug
^^^^^^^
.. attributetable:: CvDebug

.. autoclass:: CvDebug
    :members:

Image_Publisher
^^^^^^^^^^^^^^^
.. attributetable:: Image_Publisher

.. autoclass:: Image_Publisher
    :members:

Image_Subscriber
^^^^^^^^^^^^^^^^
.. attributetable:: Image_Subscriber

.. autoclass:: Image_Subscriber
    :members:

StereoImageSubscriber
^^^^^^^^^^^^^^^^^^^^^

.. attributetable:: StereoImageSubscriber

.. autoclass:: StereoImageSubscriber
    :members:

Plotter
^^^^^^^

.. attributetable:: Plotter

.. autoclass:: Plotter
    :members:

Serial
------

NoopSerial
^^^^^^^^^^

.. attributetable:: NoopSerial

.. autoclass:: NoopSerial
    :members:

SimulatedSerial
^^^^^^^^^^^^^^^
.. attributetable:: SimulatedSerial

.. autoclass:: SimulatedSerial
    :members:

txros
-----

.. currentmodule:: txros

Goal
^^^^

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

VectorToMarker
^^^^^^^^^^^^^^
.. attributetable:: mil_ros_tools.vector_to_marker.VectorToMarker

.. autoclass:: mil_ros_tools.vector_to_marker.VectorToMarker
    :members:

ClickedPointRecorder
^^^^^^^^^^^^^^^^^^^^
.. attributetable:: nodes.clicked_point_recorder.ClickedPointRecorder

.. autoclass:: nodes.clicked_point_recorder.ClickedPointRecorder
    :members:

NetworkBroadcaster
^^^^^^^^^^^^^^^^^^
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

HandlerBase
~~~~~~~~~~~
.. currentmodule:: ros_alarms

.. attributetable:: ros_alarms.HandlerBase

.. autoclass:: ros_alarms.HandlerBase
    :members:

   
C++
^^^

AlarmProxy
~~~~~~~~~~
.. cppattributetable:: ros_alarms::AlarmProxy

.. doxygenstruct:: ros_alarms::AlarmProxy

AlarmBroadcaster
~~~~~~~~~~~~~~~~
.. cppattributetable:: ros_alarms::AlarmBroadcaster

.. doxygenclass:: ros_alarms::AlarmBroadcaster

AlarmListener
~~~~~~~~~~~~~
.. cppattributetable:: ros_alarms::AlarmListener

.. doxygenclass:: ros_alarms::AlarmListener

ListenerCb
~~~~~~~~~~
.. cppattributetable:: ros_alarms::ListenerCb

.. doxygenstruct:: ros_alarms::ListenerCb

HeartbeatMonitor
~~~~~~~~~~~~~~~~
.. cppattributetable:: ros_alarms::HeartbeatMonitor

.. doxygenclass:: ros_alarms::HeartbeatMonitor

Subjugator-specific
^^^^^^^^^^^^^^^^^^^

BusVoltage
~~~~~~~~~~
.. attributetable:: alarm_handlers.BusVoltage

.. autoclass:: alarm_handlers.BusVoltage
    :members:

HeightOverBottom
~~~~~~~~~~~~~~~~
.. attributetable:: alarm_handlers.HeightOverBottom

.. autoclass:: alarm_handlers.HeightOverBottom
    :members:

HwKill
~~~~~~
.. attributetable:: alarm_handlers.HwKill

.. autoclass:: alarm_handlers.HwKill
    :members:

Kill
~~~~
.. attributetable:: alarm_handlers.Kill

.. autoclass:: alarm_handlers.Kill
    :members:

NetworkLoss
~~~~~~~~~~~
.. attributetable:: alarm_handlers.NetworkLoss

.. autoclass:: alarm_handlers.NetworkLoss
    :members:

OdomKill
~~~~~~~~
.. attributetable:: alarm_handlers.OdomKill

.. autoclass:: alarm_handlers.OdomKill
    :members:

ThrusterOut
~~~~~~~~~~~~
.. attributetable:: alarm_handlers.ThrusterOut

.. autoclass:: alarm_handlers.ThrusterOut
    :members:

Path Planning
-------------

Constraints
^^^^^^^^^^^

.. currentmodule:: lqrrt

.. attributetable:: lqrrt.Constraints

.. autoclass:: lqrrt.Constraints
    :members:

Planner
^^^^^^^

.. currentmodule:: lqrrt

.. attributetable:: lqrrt.Planner

.. autoclass:: lqrrt.Planner
    :members:
   
Tree
^^^^

.. currentmodule:: lqrrt

.. attributetable:: lqrrt.Tree

.. autoclass:: lqrrt.Tree
    :members:

Node
^^^^
.. currentmodule:: lqrrt

.. attributetable:: navigator_path_planner.nodes.path_planner.LQRRT_Node

.. autoclass:: navigator_path_planner.nodes.path_planner.LQRRT_Node
    :members:

Battery Monitor
---------------

.. currentmodule:: navigator_battery_monitor

.. attributetable:: nodes.navigator_battery_monitor.BatteryMonitor

.. autoclass:: nodes.navigator_battery_monitor.BatteryMonitor
    :members:

Passive Sonar
-------------

.. currentmodule:: mil_passive_sonar

Utility Functions
^^^^^^^^^^^^^^^^^

.. autofunction:: mil_passive_sonar.algorithms.run

.. autofunction:: mil_passive_sonar.algorithms.zero_mean

.. autofunction:: mil_passive_sonar.algorithms.normalize

.. autofunction:: mil_passive_sonar.algorithms.compute_freq

.. autofunction:: mil_passive_sonar.algorithms.bin_to_freq

.. autofunction:: mil_passive_sonar.algorithms.freq_to_bin

.. autofunction:: mil_passive_sonar.algorithms.preprocess

.. autofunction:: mil_passive_sonar.algorithms.bandpass

.. autofunction:: mil_passive_sonar.algorithms.compute_deltas

.. autofunction:: mil_passive_sonar.algorithms.make_template

.. autofunction:: mil_passive_sonar.algorithms.match_template

.. autofunction:: mil_passive_sonar.algorithms.calculate_error

.. autofunction:: mil_passive_sonar.algorithms.find_minimum

.. autofunction:: mil_passive_sonar.algorithms.compute_pos_4hyd

TxHydrophonesClient
^^^^^^^^^^^^^^^^^^^
.. attributetable:: TxHydrophonesClient

.. autoclass:: TxHydrophonesClient
    :members:

StreamedBandpass
^^^^^^^^^^^^^^^^
.. attributetable:: mil_passive_sonar.streamed_bandpass.StreamedBandpass

.. autoclass:: mil_passive_sonar.streamed_bandpass.StreamedBandpass
    :members:

HydrophoneTrigger
^^^^^^^^^^^^^^^^^
.. attributetable:: mil_passive_sonar.scripts.triggering.HydrophoneTrigger

.. autoclass:: mil_passive_sonar.scripts.triggering.HydrophoneTrigger
    :members:

PingLocator
^^^^^^^^^^^
.. attributetable:: mil_passive_sonar.scripts.ping_locator.PingLocator

.. autoclass:: mil_passive_sonar.scripts.ping_locator.PingLocator
    :members:

SylphaseSonarToRosNode
^^^^^^^^^^^^^^^^^^^^^^
.. cppattributetable:: SylphaseSonarToRosNode

.. doxygenclass:: SylphaseSonarToRosNode

CAN Communication
-----------------

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
