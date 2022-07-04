Services
--------

Mission Systems
^^^^^^^^^^^^^^^

AcousticBeacon
~~~~~~~~~~~~~~

.. attributetable:: navigator_msgs.srv.AcousticBeaconRequest

.. class:: navigator_msgs.srv.AcousticBeaconRequest

   The request class for the ``navigator_msgs/AcousticBeacon`` service. The class
   contains no settable attributes.

.. attributetable:: navigator_msgs.srv.AcousticBeaconResponse

.. class:: navigator_msgs.srv.AcousticBeaconResponse

   The repsonse class for the ``navigator_msgs/AcousticBeacon`` service.

   .. attribute:: beacon_position

        The position of the acoustic beacon.

        :type: Point

   .. attribute:: setValue

        Whether the position data of the beacon is reliable enough to be used.

        :type: bool

ChooseAnimal
~~~~~~~~~~~~
.. attributetable:: navigator_msgs.srv.ChooseAnimalRequest

.. class:: navigator_msgs.srv.ChooseAnimalRequest

   The request class for the ``navigator_msgs/ChooseAnimal`` service.

   .. attribute:: target_animal

        The target animal to circle around. Should be ``platyus``, ``crocodile``,
        or ``turtle``.

        :type: str

   .. attribute:: circle_direction

        The direction to circle in. Should be ``clockwise`` or ``anti-clockwise``.

        :type: str

.. attributetable:: navigator_msgs.srv.ChooseAnimalResponse

.. class:: navigator_msgs.srv.ChooseAnimalResponse

   The repsonse class for the ``navigator_msgs/ChooseAnimal`` service.

   .. attribute:: movement_complete

        Whether the movement was completed.

        :type: bool

ColorRequest
~~~~~~~~~~~~
.. attributetable:: navigator_msgs.srv.ColorRequestRequest

.. class:: navigator_msgs.srv.ColorRequestRequest

   The request class for the ``navigator_msgs/ColorRequest`` service.

   .. attribute:: color

        The color used to find objects with.

        :type: str

.. attributetable:: navigator_msgs.srv.ColorRequestResponse

.. class:: navigator_msgs.srv.ColorRequestResponse

   The repsonse class for the ``navigator_msgs/ColorRequest`` service.

   .. attribute:: found
    
        Whether objects were found.

        :type: bool

   .. attribute:: ids
    
        The IDs of objects that were found.

        :type: List[int]

FindPinger
~~~~~~~~~~
.. attributetable:: navigator_msgs.srv.FindPingerRequest

.. class:: navigator_msgs.srv.FindPingerRequest

   The request class for the ``navigator_msgs/FindPinger`` service. The request
   has no individual attributes.

.. attributetable:: navigator_msgs.srv.FindPingerResponse

.. class:: navigator_msgs.srv.FindPingerResponse

   The repsonse class for the ``navigator_msgs/FindPinger`` service.

   .. attribute:: pinger_position
    
        The position of the pinger.

        :type: Point

   .. attribute:: num_samples
    
        ???

        :type: int

GetDockBays
~~~~~~~~~~~
.. attributetable:: navigator_msgs.srv.GetDockBaysRequest

.. class:: navigator_msgs.srv.GetDockBaysRequest

   The request class for the ``navigator_msgs/GetDockBays`` service. The request
   has no individual attributes.

.. attributetable:: navigator_msgs.srv.GetDockBaysResponse

.. class:: navigator_msgs.srv.GetDockBaysResponse

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
    
        If :attr:`~navigator_msgs.srv.GetDockBays.success` is ``False``,
        then a message describing what went wrong.

        :type: str

GetDockShape
~~~~~~~~~~~~
.. attributetable:: navigator_msgs.srv.GetDockShapeRequest

.. class:: navigator_msgs.srv.GetDockShapeRequest

   The request class for the ``navigator_msgs/GetDockShape`` service.

   .. attribute:: Shape

        The shape to the get the associated dock of. Likely one of the associated shape
        enumerations.

        :type: str

   .. attribute:: CROSS

        Constant string attribute used to represent a cross shape on a dock. True value
        is set to ``CROSS``.

        Likely used in the :attr:`~navigator_msgs.srv.GetDockShapeRequest.Shape` attribute.

        :type: str

   .. attribute:: TRIANGLE

        Constant string attribute used to represent a triangle shape on a dock. True value
        is set to ``TRIANGLE``.

        Likely used in the :attr:`~navigator_msgs.srv.GetDockShapeRequest.Shape` attribute.

        :type: str

   .. attribute:: CIRCLE

        Constant string attribute used to represent a circle shape on a dock. True value
        is set to ``CIRCLE``.

        Likely used in the :attr:`~navigator_msgs.srv.GetDockShapeRequest.Shape` attribute.

        :type: str

   .. attribute:: Color

        The color to the get the associated dock of. Likely one of the associated color
        enumerations.

        :type: str

   .. attribute:: RED

        Constant string attribute used to represent a red shape on a dock. True value
        is set to ``RED``.

        Likely used in the :attr:`~navigator_msgs.srv.GetDockShapeRequest.Color` attribute.

        :type: str

   .. attribute:: BLUE

        Constant string attribute used to represent a triangle shape on a dock. True value
        is set to ``BLUE``.

        Likely used in the :attr:`~navigator_msgs.srv.GetDockShapeRequest.Color` attribute.

        :type: str

   .. attribute:: GREEN

        Constant string attribute used to represent a circle shape on a dock. True value
        is set to ``GREEN``.

        Likely used in the :attr:`~navigator_msgs.srv.GetDockShapeRequest.Color` attribute.

        :type: str

   .. attribute:: ANY

        Constant string attribute used to represent any value for a specific field - ie, a
        dock with any shape or color representation. Actual value is ``ANY``.

        :type: str

.. attributetable:: navigator_msgs.srv.GetDockShapeResponse

.. class:: navigator_msgs.srv.GetDockShapeResponse

   The repsonse class for the ``navigator_msgs/GetDockShape`` service.

   .. attribute:: symbol

        The associated shape and color of the returned dock.

        :type: DockShape

   .. attribute:: found
    
        Whether a viable dock was found.

        :type: bool

   .. attribute:: error
    
        If :attr:`~navigator_msgs.srv.GetDockShapeResponse.found` was false,
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
.. attributetable:: navigator_msgs.srv.GetDockShapesRequest

.. class:: navigator_msgs.srv.GetDockShapesRequest

   The request class for the ``navigator_msgs/GetDockShapes`` service.

   .. attribute:: Shape

        The shape to the get the associated dock of. Likely one of the associated shape
        enumerations.

        :type: str

   .. attribute:: CROSS

        Constant string attribute used to represent a cross shape on a dock. True value
        is set to ``CROSS``.

        Likely used in the :attr:`~navigator_msgs.srv.GetDockShapeRequest.Shape` attribute.

        :type: str

   .. attribute:: TRIANGLE

        Constant string attribute used to represent a triangle shape on a dock. True value
        is set to ``TRIANGLE``.

        Likely used in the :attr:`~navigator_msgs.srv.GetDockShapeRequest.Shape` attribute.

        :type: str

   .. attribute:: CIRCLE

        Constant string attribute used to represent a circle shape on a dock. True value
        is set to ``CIRCLE``.

        Likely used in the :attr:`~navigator_msgs.srv.GetDockShapeRequest.Shape` attribute.

        :type: str

   .. attribute:: Color

        The color to the get the associated dock of. Likely one of the associated color
        enumerations.

        :type: str

   .. attribute:: RED

        Constant string attribute used to represent a red shape on a dock. True value
        is set to ``RED``.

        Likely used in the :attr:`~navigator_msgs.srv.GetDockShapeRequest.Color` attribute.

        :type: str

   .. attribute:: BLUE

        Constant string attribute used to represent a triangle shape on a dock. True value
        is set to ``BLUE``.

        Likely used in the :attr:`~navigator_msgs.srv.GetDockShapeRequest.Color` attribute.

        :type: str

   .. attribute:: GREEN

        Constant string attribute used to represent a circle shape on a dock. True value
        is set to ``GREEN``.

        Likely used in the :attr:`~navigator_msgs.srv.GetDockShapeRequest.Color` attribute.

        :type: str

   .. attribute:: ANY

        Constant string attribute used to represent any value for a specific field - ie, a
        dock with any shape or color representation. Actual value is ``ANY``.

        :type: str

.. attributetable:: navigator_msgs.srv.GetDockShapesResponse

.. class:: navigator_msgs.srv.GetDockShapesResponse

   The repsonse class for the ``navigator_msgs/GetDockShapes`` service.

   .. attribute:: shapes

        The relevant dock shapes that were found.

        :type: List[DockShape]

   .. attribute:: found
    
        If one or more suitable shapes was returned, then true.

        :type: bool

   .. attribute:: error
    
        If :attr:`~navigator_msgs.srv.GetDockShapesResponse.found`
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

ShooterManual
~~~~~~~~~~~~~
.. attributetable:: navigator_msgs.srv.ShooterManualRequest

.. class:: navigator_msgs.srv.ShooterManualRequest

   The request class for the ``navigator_msgs/ShooterManual`` service.

   .. attribute:: feeder

        ???

        :type: float

   .. attribute:: shooter

        ???

        :type: float

.. attributetable:: navigator_msgs.srv.ShooterManualResponse

.. class:: navigator_msgs.srv.ShooterManualResponse

   The repsonse class for the ``navigator_msgs/ShooterManual`` service.

   .. attribute:: success

        Whether the shooter operation was successful.

        :type: bool

StartGate
~~~~~~~~~
.. attributetable:: navigator_msgs.srv.StartGateRequest

.. class:: navigator_msgs.srv.StartGateRequest

   The request class for the ``navigator_msgs/StartGate`` service. The request
   class no public attributes.

.. attributetable:: navigator_msgs.srv.StartGateResponse

.. class:: navigator_msgs.srv.StartGateResponse

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

.. attributetable:: ros_alarms.srv.AlarmGetRequest

.. class:: ros_alarms.srv.AlarmGetRequest

   The request class for the ``ros_alarms/AlarmGet`` service.

   .. attribute:: alarm_name

        The name of the alarm to request data about.

        :type: str

.. attributetable:: ros_alarms.srv.AlarmGetResponse

.. class:: ros_alarms.srv.AlarmGetResponse

   The repsonse class for the ``ros_alarms/AlarmGet`` service.

   .. attribute:: header

        The header for the response.

        :type: Header

   .. attribute:: alarm

        The response data about the requested alarm.

        :type: ~ros_alarms.msg.Alarm

AlarmSet
~~~~~~~~

.. attributetable:: ros_alarms.srv.AlarmSetRequest

.. class:: ros_alarms.srv.AlarmSetRequest

   The request class for the ``ros_alarms/AlarmSet`` service.

   .. attribute:: alarm

        The alarm to set.

        :type: ~ros_alarms.msg.Alarm

.. attributetable:: ros_alarms.srv.AlarmSetResponse

.. class:: ros_alarms.srv.AlarmSetResponse

   The repsonse class for the ``ros_alarms/AlarmSet`` service.

   .. attribute:: succeed

        Whether the request succeeded.

        :type: bool

CameraDBQuery
~~~~~~~~~~~~~
.. attributetable:: navigator_msgs.srv.CameraDBQueryRequest

.. class:: navigator_msgs.srv.CameraDBQueryRequest

   The request class for the ``navigator_msgs/CameraDBQuery`` service.

   .. attribute:: name

        The name of the object to query.

        :type: str

   .. attribute:: id

        The ID of the object to query.

        :type: int

.. attributetable:: navigator_msgs.srv.CameraDBQueryResponse

.. class:: navigator_msgs.srv.CameraDBQueryResponse

   The repsonse class for the ``navigator_msgs/CameraDBQuery`` service.

   .. attribute:: found

        Whether the object is found.

        :type: bool

MoveToWaypoint
~~~~~~~~~~~~~~
.. attributetable:: navigator_msgs.srv.MoveToWaypointRequest

.. class:: navigator_msgs.srv.MoveToWaypointRequest

   The request class for the ``navigator_msgs/MoveToWaypoint`` service.

   .. attribute:: target_p

        The target pose to head toward.

        :type: Pose

.. attributetable:: navigator_msgs.srv.MoveToWaypointResponse

.. class:: navigator_msgs.srv.MoveToWaypointResponse

   The repsonse class for the ``navigator_msgs/MoveToWaypoint`` service.

   .. attribute:: success

        Whether the movement was successful.

        :type: bool

ObjectDBQuery
~~~~~~~~~~~~~
.. attributetable:: navigator_msgs.srv.ObjectDBQueryRequest

.. class:: navigator_msgs.srv.ObjectDBQueryRequest

   The request class for the ``navigator_msgs/ObjectDBQuery`` service.

   .. attribute:: name

        The name of the object to find in the database.

        :type: str

   .. attribute:: cmd

        The command to run in the database. The command should be formatted as
        ``ID=YYY``, where ``ID`` is the property ID of the object to change, and
        ``YYY`` is the value to set.

        :type: str

.. attributetable:: navigator_msgs.srv.ObjectDBQueryResponse

.. class:: navigator_msgs.srv.ObjectDBQueryResponse

   The repsonse class for the ``navigator_msgs/ObjectDBQuery`` service.

   .. attribute:: found

        Whether the requested object was found.

        :type: bool

   .. attribute:: objects

        A list of all objects found.

        :type: List[PerceptionObject]

SetFrequency
~~~~~~~~~~~~
.. attributetable:: navigator_msgs.srv.SetFrequencyRequest

.. class:: navigator_msgs.srv.SetFrequencyRequest

   The request class for the ``navigator_msgs/SetFrequency`` service.

   .. attribute:: frequency

        The frequency to set.

        :type: int

.. attributetable:: navigator_msgs.srv.SetFrequencyResponse

.. class:: navigator_msgs.srv.SetFrequencyResponse

   The repsonse class for the ``navigator_msgs/SetFrequency`` service. The
   class no public attributes.

SetROI
~~~~~~
.. attributetable:: navigator_msgs.srv.SetROIRequest

.. class:: navigator_msgs.srv.SetROIRequest

   The request class for the ``navigator_msgs/SetROI`` service.

   .. attribute:: roi

        The region of interest to set.

        :type: RegionOfInterest

.. attributetable:: navigator_msgs.srv.SetROIResponse

.. class:: navigator_msgs.srv.SetROIResponse

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
.. attributetable:: navigator_msgs.srv.StereoShapeDetectorRequest

.. class:: navigator_msgs.srv.StereoShapeDetectorRequest

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

.. attributetable:: navigator_msgs.srv.StereoShapeDetectorResponse

.. class:: navigator_msgs.srv.StereoShapeDetectorResponse

   The repsonse class for the ``navigator_msgs/StereoShapeDetector`` service.

   .. attribute:: success

        Whether the detector was succesful in detecting!

        :type: bool

VisionRequest
~~~~~~~~~~~~~
.. attributetable:: navigator_msgs.srv.VisionRequestRequest

.. class:: navigator_msgs.srv.VisionRequestRequest

   The request class for the ``navigator_msgs/VisionRequest`` service.

   .. attribute:: target_name

        The target to look for in the vision system.

        :type: str
        
.. attributetable:: navigator_msgs.srv.VisionRequestResponse

.. class:: navigator_msgs.srv.VisionRequestResponse

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

.. attributetable:: std_srvs.srv.SetBoolRequest

.. class:: std_srvs.srv.SetBoolRequest

    The request type for the ``SetBool`` service. Requests for some boolean value
    to be set.

.. attributetable:: std_srvs.srv.SetBoolResponse

.. class:: std_srvs.srv.SetBoolResponse

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
.. attributetable:: navigator_msgs.srv.CameraToLidarTransformRequest

.. class:: navigator_msgs.srv.CameraToLidarTransformRequest

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

.. attributetable:: navigator_msgs.srv.CameraToLidarTransformResponse

.. class:: navigator_msgs.srv.CameraToLidarTransformResponse

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

.. attributetable:: navigator_msgs.srv.CoordinateConversionRequest

.. class:: navigator_msgs.srv.CoordinateConversionRequest

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

.. attributetable:: navigator_msgs.srv.CoordinateConversionResponse

.. class:: navigator_msgs.srv.CoordinateConversionResponse

   The repsonse class for the ``navigator_msgs/CoordinateConversion`` service.

   .. attribute:: converted
    
        The list of converted points.

        :type: List[Point]

   .. attribute:: message
    
        If an error occurred, the message of what went wrong.

        :type: str

KeyboardControl
^^^^^^^^^^^^^^^
.. attributetable:: navigator_msgs.srv.KeyboardControlRequest

.. class:: navigator_msgs.srv.KeyboardControlRequest

   The request class for the ``navigator_msgs/KeyboardControl`` service.

   .. attribute:: uuid

        A unique ID to represent the process (?).

        :type: str

   .. attribute:: keycode

        The keycode that was pressed.

        :type: int

.. attributetable:: navigator_msgs.srv.KeyboardControlResponse

.. class:: navigator_msgs.srv.KeyboardControlResponse

   The repsonse class for the ``navigator_msgs/KeyboardControl`` service.

   .. attribute:: generated_uuid

        A response unique ID that was generated in response to the request.

        :type: str

   .. attribute:: is_locked
    
        Whether the client which sent the keycode has "locked control" of the keyboard
        server, and is therefore blocking other keyboard input.

        :type: bool

POI Handling
^^^^^^^^^^^^

AddPOI
~~~~~~
.. attributetable:: mil_poi.srv.AddPOIRequest

.. class:: mil_poi.srv.AddPOIRequest

   The request class for the ``mil_poi/AddPOI`` service.

   .. attribute:: name

        The name of the POI to add.

        :type: str

   .. attribute:: position

        The position of the new POI.

        :type: PointStamped

.. attributetable:: mil_poi.srv.AddPOIResponse

.. class:: mil_poi.srv.AddPOIResponse

   The repsonse class for the ``mil_poi/AddPOI`` service.

   .. attribute:: success

        Whether the add operation was succesful.

        :type: bool

   .. attribute:: message

        The message associated with the success of the add operation.

        :type: str

MovePOI
~~~~~~~
.. attributetable:: mil_poi.srv.MovePOIRequest

.. class:: mil_poi.srv.MovePOIRequest

   The request class for the ``mil_poi/MovePOI`` service.

   .. attribute:: name

        The name of the POI to move.

        :type: str

   .. attribute:: position

        The position of the new POI.

        :type: PointStamped

.. attributetable:: mil_poi.srv.MovePOIResponse

.. class:: mil_poi.srv.MovePOIResponse

   The repsonse class for the ``mil_poi/MovePOI`` service.

   .. attribute:: success

        Whether the move operation was succesful.

        :type: bool

   .. attribute:: message

        The message associated with the success of the move operation.

        :type: str

DeletePOI
~~~~~~~~~
.. attributetable:: mil_poi.srv.DeletePOIRequest

.. class:: mil_poi.srv.DeletePOIRequest

   The request class for the ``mil_poi/DeletePOI`` service.

   .. attribute:: name

        The name of the POI to delete.

        :type: str

.. attributetable:: mil_poi.srv.DeletePOIResponse

.. class:: mil_poi.srv.DeletePOIResponse

   The repsonse class for the ``mil_poi/DeletePOI`` service.

   .. attribute:: success

        Whether the delete operation was succesful.

        :type: bool

   .. attribute:: message

        The message associated with the success of the delete operation.

        :type: str
