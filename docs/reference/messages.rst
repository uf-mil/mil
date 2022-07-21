Messages
--------

actionlib
^^^^^^^^^

GoalStatus
~~~~~~~~~~

.. attributetable:: actionlib_msgs.msg.GoalStatus

.. class:: actionlib_msgs.msg.GoalStatus

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

.. attributetable:: ros_alarms.msg.Alarm

.. class:: ros_alarms.msg.Alarm

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

.. attributetable:: geometry_msgs.msg.Quaternion

.. class:: geometry_msgs.msg.Quaternion

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

.. attributetable:: geometry_msgs.msg.Point

.. class:: geometry_msgs.msg.Point

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

PointStamped
~~~~~~~~~~~~

.. attributetable:: geometry_msgs.msg.PointStamped

.. class:: geometry_msgs.msg.PointStamped

    A ROS message type representing a single PointStamped.

    .. attribute:: header

        The header associated with the point.

        :type: :class:`Header`

    .. attribute:: position

        The position associated with the point

        :type: :class:`Point`

Vector3
~~~~~~~

.. attributetable:: geometry_msgs.msg.Vector3

.. class:: geometry_msgs.msg.Vector3

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

.. attributetable:: geometry_msgs.msg.Pose

.. class:: geometry_msgs.msg.Pose

    A ROS message type representing an object's pose.

    .. attribute:: position

        The position of the pose.

        :type: ~geometry_msgs.msg.Pose

    .. attribute:: orientation

        The orientation of the pose.

        :type: ~geometry_msgs.msg.Quaternion

Pose2D
~~~~~~

.. attributetable:: geometry_msgs.msg.Pose2D

.. class:: geometry_msgs.msg.Pose2D

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

.. attributetable:: geometry_msgs.msg.PoseWithCovariance

.. class:: geometry_msgs.msg.PoseWithCovariance

    A ROS message type representing an object's pose, along with a covariance.

    .. attribute:: pose

        The object's pose.

        :type: ~geometry_msgs.msg.Pose

    .. attribute:: covariance

        The object's covariance. Consists of a list of 36 values.

        :type: List[float]

Transform
~~~~~~~~~

.. attributetable:: geometry_msgs.msg.Transform

.. class:: geometry_msgs.msg.Transform

    A ROS message type representing an object's transform.

    .. attribute:: translation

        The translation of the transform.

        :type: ~geometry_msgs.msg.Vector3

    .. attribute:: rotation

        The rotation of the transform.

        :type: ~geometry_msgs.msg.Quaternion

TransformStamped
~~~~~~~~~~~~~~~~

.. attributetable:: geometry_msgs.msg.TransformStamped

.. class:: geometry_msgs.msg.TransformStamped

    A stamped ROS message type representing an object's transform.

    .. attribute:: header

        The header of the message.

        :type: ~std_msgs.msg.Header

    .. attribute:: child_frame_id

        The ID of the child frame.

        :type: str

    .. attribute:: transform

        The transform in the message.

        :type: ~geometry_msgs.msg.Transform

Accel
~~~~~

.. attributetable:: geometry_msgs.msg.Accel

.. class:: geometry_msgs.msg.Accel

    A ROS message type representing an object's acceleration.

    .. attribute:: linear

        The linear acceleration of the twist.

        :type: ~geometry_msgs.msg.Vector3

    .. attribute:: angular

        The angular acceleration of the twist.

        :type: ~geometry_msgs.msg.Vector3

Twist
~~~~~

.. attributetable:: geometry_msgs.msg.Twist

.. class:: geometry_msgs.msg.Twist

    A ROS message type representing an object's twist.

    .. attribute:: linear

        The linear direction of the twist.

        :type: ~geometry_msgs.msg.Vector3

    .. attribute:: angular

        The angular direction of the twist.

        :type: ~geometry_msgs.msg.Vector3

TwistWithCovariance
~~~~~~~~~~~~~~~~~~~

.. attributetable:: geometry_msgs.msg.TwistWithCovariance

.. class:: geometry_msgs.msg.TwistWithCovariance

    A ROS message type representing an object's twist, along with a covariance.

    .. attribute:: twist

        The object's twist.

        :type: ~geometry_msgs.msg.Twist

    .. attribute:: covariance

        The object's covariance. Consists of a list of 36 values.

        :type: List[float]

Polygon
~~~~~~~

.. attributetable:: geometry_msgs.msg.Polygon

.. class:: geometry_msgs.msg.Polygon

    A ROS message type representing a polygon.

    .. attribute:: points

        The points constructing the polygon.

        :type: List[~geometry_msgs.msg.Point]

Wrench
~~~~~~

.. attributetable:: geometry_msgs.msg.Wrench

.. class:: geometry_msgs.msg.Wrench

    A ROS message type representing the wrench of an object.

    .. attribute:: force

        The force associated with the object.

        :type: ~geometry_msgs.msg.Vector3

    .. attribute:: torque

        The torque associated with the object.

        :type: ~geometry_msgs.msg.Vector3

WrenchStamped
~~~~~~~~~~~~~

.. attributetable:: geometry_msgs.msg.WrenchStamped

.. class:: geometry_msgs.msg.WrenchStamped

    A ROS message type representing the wrench of an object with an associated header.

    .. attribute:: header

        The header associated with the message.

        :type: ~std_msgs.msg.Header

    .. attribute:: wrench

        The wrench associated with the object.

        :type: ~geometry_msgs.msg.Wrench

MIL Messages
^^^^^^^^^^^^

PoseTwist
~~~~~~~~~

.. attributetable:: mil_msgs.msg.PoseTwist

.. class:: mil_msgs.msg.PoseTwist

    A ROS message type representing an object's pose and twist.

    .. attribute:: pose

        The pose of the object.

        :type: ~geometry_msgs.msg.Pose

    .. attribute:: twist

        The twist of the object.

        :type: ~geometry_msgs.msg.Twist

    .. attribute:: acceleration

        The acceleration of the object.

        :type: ~geometry_msgs.msg.Accel

ObjectInImage
~~~~~~~~~~~~~

.. attributetable:: mil_msgs.msg.ObjectInImage

.. class:: mil_msgs.msg.ObjectInImage

    A ROS message type representing the position of an object in an image.

    .. attribute:: name

        The name of the object.

        :type: str

    .. attribute:: points

        The center of the object in the image.

        :type: ~mil_msgs.msg.Point2D

    .. attribute:: confidence

        The confidence of the object's position, ranging between 0 and 1.

        :type: float

    .. attribute:: attributes

        ???

        :type: str

Point2D
~~~~~~~

.. attributetable:: mil_msgs.msg.Point2D

.. class:: mil_msgs.msg.Point2D

    A ROS message type representing an x and y position in a 2D space.

    .. attribute:: x

        The x-position.

        :type: float

    .. attribute:: y

        The y-position.

        :type: float

Navigation Messages
^^^^^^^^^^^^^^^^^^^

Odometry
~~~~~~~~

.. attributetable:: nav_msgs.msg.Odometry

.. class:: nav_msgs.msg.Odometry

    A ROS message type representing an object's odometry.

    .. attribute:: header

        The message header.

        :type: ~std_msgs.msg.Header

    .. attribute:: child_frame_id

        The child frame ID, used to determine the frame of the robot's twist.

        :type: ~geometry_msgs.msg.Twist

    .. attribute:: pose

        The pose (along with covariance) determined within the frame of :attr:`~std_msgs.msg.Header.frame_id`.

        :type: ~geometry_msgs.msg.PoseWithCovariance

    .. attribute:: twist

        The twist (along with covariance) determined within the frame of :attr:`~nav_msgs.msg.Odometry.child_frame_id`.

        :type: ~geometry_msgs.msg.TwistWithCovariance

Acceleration
~~~~~~~~~~~~

.. attributetable:: navigator_msgs.msg.Acceleration

.. class:: navigator_msgs.msg.Acceleration

    .. attribute:: linear

        The linear component of the acceleration.

        :type: Vector3

    .. attribute:: angular

        The angular component of the acceleration.

        :type: Vector3

KillStatus
~~~~~~~~~~

.. attributetable:: navigator_msgs.msg.KillStatus

.. class:: navigator_msgs.msg.KillStatus

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

.. attributetable:: navigator_msgs.msg.PerceptionObject

.. class:: navigator_msgs.msg.PerceptionObject

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

.. attributetable:: navigator_msgs.msg.PerceptionObjectArray

.. class:: navigator_msgs.msg.PerceptionObjectArray

    A custom message definition to represent an array of perception objects.

    .. attribute:: objects

        The objects in the array.

        :type: List[PerceptionObject]

Networking Messages
^^^^^^^^^^^^^^^^^^^

Host
~~~~

.. attributetable:: navigator_msgs.msg.Host

.. class:: navigator_msgs.msg.Host

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

.. attributetable:: navigator_msgs.msg.Hosts

.. class:: navigator_msgs.msg.Hosts

    A custom message definition representing a group of hosts together.

    .. attribute:: hostnames

        A custom, constant string representing a group of hostnames. The string is
        equal to:

            mil-nav-wamv mil-nav-ubnt-wamv mil-nav-ubnt-shore mil-com-velodyne-vlp16 mil-com-sick-lms111

        :type: str

    .. attribute:: hosts

        The hosts belonging to the group.

        :type: List[~navigator_msgs.msg.Host]

Passive Sonar Messages
^^^^^^^^^^^^^^^^^^^^^^

HydrophoneSamples
~~~~~~~~~~~~~~~~~

.. attributetable:: mil_passive_sonar.msg.HydrophoneSamples

.. class:: mil_passive_sonar.msg.HydrophoneSamples

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

.. attributetable:: mil_passive_sonar.msg.HydrophoneSamplesStamped

.. class:: mil_passive_sonar.msg.HydrophoneSamplesStamped

    A custom message definition to represent time-stamped data coming from the hydrophones.

    .. attribute:: header

        The message header.

        :type: Header

    .. attribute:: hydrophone_samples

        The hydrophone samples received.

        :type: HydrophoneSamples

Ping
~~~~

.. attributetable:: mil_passive_sonar.msg.Ping

.. class:: mil_passive_sonar.msg.Ping

    A custom message definition to represent a ping from channels of data.

    .. danger::

        This class is deprecated, and has been replaced by
        :class:`~mil_passive_sonar.msg.HydrophoneSamples` and
        :class:`~mil_passive_sonar.msg.HydrophoneSamplesStamped`.
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

.. attributetable:: mil_passive_sonar.msg.ProcessedPing

.. class:: mil_passive_sonar.msg.ProcessedPing

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

.. attributetable:: mil_passive_sonar.msg.Triggered

.. class:: mil_passive_sonar.msg.Triggered

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

.. attributetable:: std_msgs.msg.Header

.. class:: std_msgs.msg.Header

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

.. attributetable:: std_msgs.msg.ColorRGBA

.. class:: std_msgs.msg.ColorRGBA

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

Image
~~~~~

.. attributetable:: sensor_msgs.msg.Image

.. class:: sensor_msgs.msg.Image

    A ROS message to represent an image.

    .. attribute:: header

        The header associated with the message.

        :type: Header

    .. attribute:: height

        The height of the image.

        :type: int

    .. attribute:: width

        The width of the image.

        :type: int

    .. attribute:: encoding

        The encoding of the image.

        :type: str

    .. attribute:: is_bigendian

        Whether the image uses the big-endian format to store values.

        :type: bool

    .. attribute:: step

        Full length of a row, in bytes.

        :type: int

    .. attribute:: data

        Actual image data. The size of the list is equal to :attr:`.step` multiplied
        by the number of rows.

        :type: List[int]

PointField
~~~~~~~~~~

.. attributetable:: sensor_msgs.msg.PointField

.. class:: sensor_msgs.msg.PointField

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

.. attributetable:: sensor_msgs.msg.PointCloud2

.. class:: sensor_msgs.msg.PointCloud2

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

        The actual data inside the point cloud. The size of the array is :attr:`~sensor_msgs.msg.PointCloud2.row_step`
        multiplied by :attr:`~sensor_msgs.msg.PointCloud2.height`.

        :type: List[int]

    .. attribute:: is_dense

        ``True`` if there are no invalid points.

        :type: bool

Motor Feedback
~~~~~~~~~~~~~~
.. attributetable:: roboteq_msgs.msg.Feedback

.. class:: roboteq_msgs.msg.Feedback

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
.. attributetable:: roboteq_msgs.msg.Status

.. class:: roboteq_msgs.msg.Status

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

.. attributetable:: navigator_msgs.msg.DockShape

.. class:: navigator_msgs.msg.DockShape

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

        :type: List[~geometry_msgs.msg.Point]

    .. attribute:: color_confidence

        The relative confidence that the color prediction is correct.

        :type: float

    .. attribute:: shape_confidence

        The relative confidence that the shape prediction is correct.

        :type: float

DockShapes
~~~~~~~~~~

.. attributetable:: navigator_msgs.msg.DockShapes

.. class:: navigator_msgs.msg.DockShapes

    A custom message definition to represent the presence of multiple dock shapes
    found by the vision system.

    .. attribute:: list

        The list of shapes found.

        :type: List[DockShape]


ScanTheCode
~~~~~~~~~~~

.. attributetable:: navigator_msgs.msg.ScanTheCode

.. class:: navigator_msgs.msg.ScanTheCode

    A custom message definition to represent the color pattern show by a Scan The
    Code totem.

    .. attribute:: string_pattern

        The pattern shown. ``R`` stands for red, ``B`` stands for blue, and ``G``
        stands for green.

        :type: str
