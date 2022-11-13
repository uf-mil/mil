#!/usr/bin/env python3

from threading import Lock
from typing import Optional

import rospy
import tf2_ros
from geometry_msgs.msg import Point, PointStamped, Pose, Quaternion
from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from mil_poi.msg import POI, POIArray
from mil_poi.srv import (
    AddPOI,
    AddPOIRequest,
    AddPOIResponse,
    DeletePOI,
    DeletePOIRequest,
    DeletePOIResponse,
    MovePOI,
    MovePOIRequest,
    MovePOIResponse,
)
from mil_ros_tools.msg_helpers import numpy_to_point
from mil_tools import thread_lock
from std_srvs.srv import Trigger, TriggerRequest, TriggerResponse
from visualization_msgs.msg import (
    InteractiveMarker,
    InteractiveMarkerControl,
    InteractiveMarkerFeedback,
    Marker,
)

# Mutex for POIServer
lock = Lock()


class POIServer:
    """
    Node to act as a server to hold a list of points of interest which
    can be modified by services or interactive markers.

    Attributes:
        marker_scale (float): Radius of interactive markers for POIs. Defaults to ``0.5``.
    """

    def __init__(self):
        # TF bootstrap
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # Radius of interactive marker for POIs
        self.marker_scale = rospy.get_param("~marker_scale", 0.5)

        # Create initial empty POI array
        self.pois = POIArray()

        # Get the global frame to be used
        self.global_frame = rospy.get_param("~global_frame", "map")
        self.pois.header.frame_id = self.global_frame

        # Create publisher to notify clients of updates and interactive marker server
        self.pub = rospy.Publisher(
            "points_of_interest", POIArray, queue_size=1, latch=True
        )
        self.interactive_marker_server = InteractiveMarkerServer("points_of_interest")

        # Load initial POIs from params
        if rospy.has_param("~initial_pois"):
            pois = rospy.get_param("~initial_pois")
            assert isinstance(pois, dict)
            for key, value in pois.items():
                assert isinstance(key, str)
                assert isinstance(value, list)
                assert len(value) == 3
                name = key
                pose = Pose()
                pose.position = numpy_to_point(value)
                pose.orientation = Quaternion(0.0, 0.0, 0.0, 0.0)
                self._add_poi(name, pose)

        # Update clients / markers of changes from param
        self.update()

        # Create services to add / delete / move a POI
        self.add_poi_server = rospy.Service("~add", AddPOI, self.add_poi_cb)
        self.delete_poi_server = rospy.Service("~delete", DeletePOI, self.delete_poi_cb)
        self.move_poi_service = rospy.Service("~move", MovePOI, self.move_poi_cb)
        self.save_to_param = rospy.Service(
            "~save_to_param", Trigger, self.save_to_param_cb
        )

    def transform_position(self, ps: PointStamped) -> Optional[Point]:
        """
        Attempt to transform a PointStamped message into the global frame, returning
        the position of the transformed point or None if transform failed.

        Args:
            ps (PointStamped): The PointStamped message to be transformed into the
                global frame.

        Returns:
            Optional[Point]: The point in the global frame, or ``None`` if the transform
            experienced an exception.
        """
        # If no frame, assume user wanted it in the global frame
        if ps.header.frame_id == "":
            return ps.point
        try:
            ps_tf = self.tf_buffer.transform(
                ps, self.global_frame, timeout=rospy.Duration(5)
            )
            return ps_tf.point
        except tf2_ros.TransformException as e:
            rospy.logwarn(
                'Error transforming "{}" to "{}": {}'.format(
                    ps.header.frame_id, self.global_frame, e
                )
            )
            return None

    def process_feedback(self, feedback: InteractiveMarkerFeedback) -> None:
        """
        Process interactive marker feedback, moving markers internally
        in response to markers moved in Rviz.

        Args:
            feedback (InteractiveMarkerFeedback): The feedback message.
        """
        # Only look at changes when mouse button is unclicked
        if feedback.event_type != InteractiveMarkerFeedback.MOUSE_UP:
            return

        # Transform new position into global frame
        ps = PointStamped()
        ps.header = feedback.header
        ps.point = feedback.pose.position
        position = self.transform_position(ps)
        if position is None:
            return

        # Update position of marker
        self.update_position(feedback.marker_name, feedback.pose)

    @thread_lock(lock)
    def save_to_param_cb(self, req: TriggerRequest) -> TriggerResponse:
        """
        Saves the internal parameters into the parameter server. Returns whether
        the operation was successful.

        Args:
            req (TriggerRequest): The service request. Has no notable attributes.

        Returns:
            TriggerResponse: The response from the service; ie, whether the operation
            succeeded.
        """
        rospy.set_param("~global_frame", self.global_frame)
        d = {}
        for poi in self.pois.pois:
            d[poi.name] = [
                float(poi.position.x),
                float(poi.position.y),
                float(poi.position.z),
            ]
        rospy.set_param("~initial_pois", d)
        return TriggerResponse(success=True)

    def add_poi_cb(self, req: AddPOIRequest) -> AddPOIResponse:
        """
        Callback for the AddPOI service.

        Args:
            req (AddPOIRequest): The service request. The name and position are used
                from the request.

        Returns:
            AddPOIResponse: Whether the operation was successful, along with a message
            indicating why.
        """
        name = req.name
        pose = Pose()
        # If frame is empty, just initialize it to zero
        if len(req.position.header.frame_id) == 0:
            position = Point(0.0, 0.0, 0.0)

        # Otherwise transform position into global frame
        else:
            position = self.transform_position(req.position)
            if position is None:
                return AddPOIResponse(success=False, message="tf error (bad poi)")

        orientation = Quaternion(0.0, 0.0, 0.0, 0.0)
        pose.position, pose.orientation = position, orientation
        if not self.add_poi(name, pose):
            return AddPOIResponse(success=True, message="already exists (bad poi)")
        return AddPOIResponse(success=True, message="good poi")

    def delete_poi_cb(self, req: DeletePOIRequest) -> DeletePOIResponse:
        """
        Callback for DeletePOI service. Equivalent to calling :meth:`.remove_poi`.

        Args:
            req (DeletePOIRequest): The service request. The name is used to determine
                which POI to remove.

        Returns:
            DeletePOIResponse: Whether the operation was successful, along with a
            message indicating why.
        """
        # Return error if marker did not exist
        if not self.remove_poi(req.name):
            return DeletePOIResponse(success=False, message="does not exist (bad poi)")
        return DeletePOIResponse(success=True, message="good poi")

    def move_poi_cb(self, req: MovePOIRequest) -> MovePOIResponse:
        """
        Callback for MovePOI service.

        Args:
            req (MovePOIRequest): The service request.

        Returns:
            MovePOIResponse: The response from the service.
        """
        name = req.name
        # Transform position into global frame
        position = self.transform_position(req.position)
        if position is None:
            return MovePOIResponse(success=False, message="tf error (bad poi)")
        # Return error if marker did not exist
        if not self.update_position(name, Pose(position, Quaternion(0, 0, 0, 0))):
            return MovePOIResponse(success=False, message="does not exist (bad poi)")
        return MovePOIResponse(success=True, message="good poi")

    @thread_lock(lock)
    def add_poi(self, name: str, position: Pose) -> bool:
        """
        Add a POI, updating clients and Rviz when done.

        Args:
            name (str): The name of the POI to add.
            position (Point): The position of the new POI.

        Returns:
            bool: ``False`` if a POI with the same name already exists, otherwise ``True``.
        """
        if not self._add_poi(name, position):
            return False
        self.update()
        return True

    @thread_lock(lock)
    def remove_poi(self, name: str) -> bool:
        """
        Remove a POI, updating clients and Rviz when done.

        Args:
            name (str): The name of the POI to remove.

        Returns:
            bool: ``False`` if a POI with the provided name does not exist, otherwise ``True``.
        """
        if not self._remove_poi(name):
            return False
        self.update()
        return True

    @thread_lock(lock)
    def update_position(self, name: str, position: Pose) -> bool:
        """
        Update the position of a POI, updating clients and Rviz when done.

        Args:
            name (str): The name of the POI to remove.
            position (:class:`~geometry_msgs.msg._Point.Point`): A Point message
                of the new position in global frame.

        Returns:
            bool: ``False`` if a POI with name does not exist, otherwise ``True``.
        """
        if not self._update_position(name, position):
            return False
        self.update()
        return True

    def update(self) -> None:
        """
        Update interactive markers server and POI publish of changes.
        """
        self.pois.header.stamp = rospy.Time.now()
        self.pub.publish(self.pois)
        self.interactive_marker_server.applyChanges()

    def _update_position(self, name: str, position: Pose) -> bool:
        """
        Internal implementation of update_position, which is NOT thread safe
        and does NOT update clients of change.
        """
        # Find poi with specified name
        for poi in self.pois.pois:
            if poi.name == name:
                if not self.interactive_marker_server.setPose(name, position):
                    return False
                # Set pose in message
                poi.pose = position
                return True
        return False

    def _remove_poi(self, name: str) -> bool:
        """
        Internal implementation of remove_poi, which is NOT thread safe and
        does NOT update clients of change.
        """
        # Return false if marker with that name not added to interactive marker server
        if not self.interactive_marker_server.erase(name):
            return False
        # Find POI with specified name and delete it from list
        for i, poi in enumerate(self.pois.pois):
            if poi.name == name:
                del self.pois.pois[i]
                return True
        return False

    def _add_poi(self, name: str, position: Pose) -> bool:
        """
        Internal implementation of add_poi, which is NOT thread safe and does
        NOT update clients of change.
        """
        if self.interactive_marker_server.get(name) is not None:
            return False
        poi = POI()
        poi.name = name
        poi.pose = position
        self.pois.pois.append(poi)

        point_marker = Marker()
        point_marker.type = Marker.ARROW
        point_marker.scale.x = self.marker_scale * 1
        point_marker.scale.y = self.marker_scale * 0.25
        point_marker.scale.z = self.marker_scale * 0.25
        point_marker.color.r = 1.0
        point_marker.color.g = 1.0
        point_marker.color.b = 1.0
        point_marker.color.a = 1.0
        point_marker.pose.orientation.w = 1.0
        point_marker.pose.orientation.x = 0.0
        point_marker.pose.orientation.y = 1.0
        point_marker.pose.orientation.z = 0.0

        text_marker = Marker()
        text_marker.type = Marker.TEXT_VIEW_FACING
        text_marker.pose.orientation.w = 1.0
        text_marker.pose.orientation.x = 0.0
        text_marker.pose.orientation.y = 1.0
        text_marker.pose.orientation.z = 0.0
        text_marker.pose.position.x = 1.5
        text_marker.text = poi.name
        text_marker.scale.z = 1.0
        text_marker.color.r = 1.0
        text_marker.color.g = 1.0
        text_marker.color.b = 1.0
        text_marker.color.a = 1.0

        int_marker = InteractiveMarker()
        int_marker.header.frame_id = self.global_frame
        int_marker.pose.orientation.w = 1.0
        int_marker.pose.orientation.x = 0.0
        int_marker.pose.orientation.y = 1.0
        int_marker.pose.orientation.z = 0.0
        int_marker.pose.position = poi.pose.position
        int_marker.scale = 1

        int_marker.name = poi.name

        # insert a box
        control = InteractiveMarkerControl()
        control.interaction_mode = InteractiveMarkerControl.MOVE_ROTATE
        control.markers.append(point_marker)
        control.markers.append(text_marker)
        control.always_visible = True
        int_marker.controls.append(control)

        self.interactive_marker_server.insert(int_marker, self.process_feedback)

        return True
