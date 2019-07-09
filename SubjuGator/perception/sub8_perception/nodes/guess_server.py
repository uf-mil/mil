#!/usr/bin/env python
import sys
import rospy
from sub8_msgs.srv import GuessRequest, GuessRequestResponse
from geometry_msgs.msg import PoseStamped
from interactive_markers.interactive_marker_server import InteractiveMarker, InteractiveMarkerServer
from visualization_msgs.msg import Marker, InteractiveMarkerControl


class Guess:
    def __init__(self):
        rospy.sleep(1.0)
        self.items = [
            'pinger_surface', 'pinger_shooter', 'vampire_slayer'
        ]
        self.guess_service = rospy.Service('guess_location', GuessRequest,
                                           self.request_location)
        self.markers_subscribers = []
        self.markers_locations = dict.fromkeys(self.items)
        self.markers_servers = []
        self.markers = []
        box_marker = Marker()
        box_marker.type = Marker.CUBE
        box_marker.scale.x = 0.45
        box_marker.scale.y = 0.45
        box_marker.scale.z = 0.45
        box_marker.color.r = 0.0
        box_marker.color.g = 0.5
        box_marker.color.b = 0.5
        box_marker.color.a = 1.0
        box_control = InteractiveMarkerControl()
        box_control.always_visible = True
        box_control.markers.append(box_marker)
        rotate_control = InteractiveMarkerControl()
        rotate_control.name = "move_x"
        rotate_control.orientation.w = 0.707
        rotate_control.orientation.x = 0
        rotate_control.orientation.y = 0.707
        rotate_control.orientation.z = 0
        rotate_control.interaction_mode = InteractiveMarkerControl.MOVE_PLANE
        spacer = 0
        for i in self.items:
            self.markers.append(InteractiveMarker())
            self.markers[spacer].header.frame_id = "map"
            self.markers[spacer].name = i
            self.markers[spacer].description = i
            self.markers[spacer].controls.append(box_control)
            self.markers[spacer].controls.append(rotate_control)
            self.markers[spacer].pose.position.x = spacer
            self.markers[spacer].pose.position.y = 0
            self.markers[spacer].pose.position.z = 0
            spacer = spacer + 1

    def process_feedback(self, feedback):
        self.markers_locations[feedback.marker_name] = PoseStamped(
            header=feedback.header, pose=feedback.pose)

    def request_location(self, srv):
        req_item = srv.item
        if (req_item in self.items):
            return GuessRequestResponse(
                location=self.markers_locations[req_item], found=True)
        else:
            return GuessRequestResponse(found=False)


def main(args):
    server = InteractiveMarkerServer("guess_markers")
    guess = Guess()
    for i in range(len(guess.items)):
        server.insert(guess.markers[i], guess.process_feedback)
    server.applyChanges()
    rospy.spin()


if __name__ == '__main__':
    rospy.init_node('guess')
    main(sys.argv)
