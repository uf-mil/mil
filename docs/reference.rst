Reference
=========

Messages
--------

actionlib
^^^^^^^^^

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

Geometry
^^^^^^^^

.. class:: geometry_msgs.msg._Quaternion.Quaternion

    A message type representing a quaternion.

    .. attribute:: w
    
        The first element of the quaternion.

        :rtype: Union[:class:`float`, :class:`int`]

    .. attribute:: x
    
        The second element of the quaternion.

        :rtype: Union[:class:`float`, :class:`int`]

    .. attribute:: y
    
        The third element of the quaternion.

        :rtype: Union[:class:`float`, :class:`int`]

    .. attribute:: z
    
        The fourth element of the quaternion.

        :rtype: Union[:class:`float`, :class:`int`]

.. class:: geometry_msgs.msg._Pose.Point

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

.. class:: geometry_msgs.msg._Pose.Pose

    A ROS message type representing an object's pose.

    .. attribute:: position

        The position of the pose.

        :type: ~geometry_msgs.msg._Pose.Point

    .. attribute:: orientation

        The orientation of the pose.

        :type: ~geometry_msgs.msg._Quaternion.Quaternion

.. class:: geometry_msgs.msg._PoseWithCovariance.PoseWithCovariance

    A ROS message type representing an object's pose, along with a covariance.

    .. attribute:: pose

        The object's pose.

        :type: ~geometry_msgs.msg._Pose.Pose

    .. attribute:: covariance

        The object's covariance. Consists of a list of 36 values.

        :type: List[float]

.. class:: geometry_msgs.msg._Accel.Accel

    A ROS message type representing an object's acceleration.

    .. attribute:: linear

        The linear acceleration of the twist.

        :type: ~geometry_msgs.msg._Vector3.Vector3

    .. attribute:: angular

        The angular acceleration of the twist.

        :type: ~geometry_msgs.msg._Vector3.Vector3

.. class:: geometry_msgs.msg._Twist.Twist

    A ROS message type representing an object's twist.

    .. attribute:: linear

        The linear direction of the twist.

        :type: ~geometry_msgs.msg._Vector3.Vector3

    .. attribute:: angular

        The angular direction of the twist.

        :type: ~geometry_msgs.msg._Vector3.Vector3

.. class:: geometry_msgs.msg._TwistWithCovariance.TwistWithCovariance

    A ROS message type representing an object's twist, along with a covariance.

    .. attribute:: twist

        The object's twist.

        :type: ~geometry_msgs.msg._Twist.Twist

    .. attribute:: covariance

        The object's covariance. Consists of a list of 36 values.

        :type: List[float]

.. class:: geometry_msgs.msg._Twist.PoseTwist

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

.. class:: geometry_msgs.msg._Polygon.Polygon

    A ROS message type representing a polygon.

    .. attribute:: points

        The points constructing the polygon.

        :type: List[~geometry_msgs.msg._Point.Point]

Navigation Messages
^^^^^^^^^^^^^^^^^^^
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

.. autoclass:: Image_Publisher
    :members:

.. autoclass:: Image_Subscriber
    :members:

.. autoclass:: StereoImageSubscriber
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
