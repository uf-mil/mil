from .sub_singleton import SubjuGator
from txros import util
import numpy as np
from sensor_msgs.msg import CameraInfo
from image_geometry import PinholeCameraModel
import mil_ros_tools
from twisted.internet import defer
from geometry_msgs.msg import Point
from mil_misc_tools import text_effects
from std_srvs.srv import SetBool, SetBoolRequest

from sub8_msgs.srv import GuessRequest, GuessRequestRequest

import mil_ros_tools
import rospy
from std_srvs.srv import Trigger

fprint = text_effects.FprintFactory(title="BALL_DROP", msg_color="cyan").fprint

SPEED = 0.25
FAST_SPEED = 1

SEARCH_HEIGHT = 3
HEIGHT_BALL_DROPER = 2.3
TRAVEL_DEPTH = 1  # 2


class BallDrop(SubjuGator):

    @util.cancellableInlineCallbacks
    def run(self, args):
        try:
          ree = rospy.ServiceProxy('/vision/garlic/enable', SetBool)
          resp = ree(True)
          if not resp:
            print("Error, failed to init neural net.")
            return
        except rospy.ServiceException, e:
          print("Service Call Failed")

        fprint('Enabling cam_ray publisher')

        yield self.nh.sleep(1)

        fprint('Connecting camera')

        cam_info_sub = yield self.nh.subscribe(
            '/camera/down/camera_info',
            CameraInfo)

        fprint('Obtaining cam info message')
        cam_info = yield cam_info_sub.get_next_message()
        cam_center = np.array([cam_info.width/2, cam_info.height/2])
        print cam_center
        cam_norm = np.sqrt(cam_center[0]**2 + cam_center[1]**2)
        fprint('Cam center: {}'.format(cam_center))

        model = PinholeCameraModel()
        model.fromCameraInfo(cam_info)

     #   enable_service = self.nh.get_service_client("/vamp/enable", SetBool)
     #   yield enable_service(SetBoolRequest(data=True))

        try:
            position = yield self.poi.get('ball_drop')
            fprint('Found ball_drop: {}'.format(position), msg_color='green')
            yield self.move.look_at_without_pitching(position).set_position(position).depth(TRAVEL_DEPTH).go(speed=FAST_SPEED)
        except Exception as e:
            fprint(str(e) + 'Forgot to run guess server?', msg_color='yellow')

        ball_drop_sub = yield self.nh.subscribe('/bbox_pub', Point)
        yield self.move.to_height(SEARCH_HEIGHT).zero_roll_and_pitch().go(speed=SPEED)
        i = 0
        while i < 20:
            fprint('Getting location of ball drop...')
            ball_drop_msg = yield ball_drop_sub.get_next_message()
            ball_drop_xy = mil_ros_tools.rosmsg_to_numpy(ball_drop_msg)[:2]
            vec = ball_drop_xy - cam_center
            vec2 = [-vec[1], -vec[0]]

            if np.linalg.norm(vec) < 50:
                break
            fprint("Vec: {}".format(vec2))
            vec2 = vec2 / cam_norm

            fprint("Rel move vec {}".format(vec2))
            vec2 = np.append(vec2, 0)

            yield self.move.relative_depth(vec2).go(speed=SPEED)
            i+=1

        fprint('Centered, going to depth {}'.format(HEIGHT_BALL_DROPER))
        yield self.move.to_height(HEIGHT_BALL_DROPER).zero_roll_and_pitch().go(speed=SPEED)
        fprint('Dropping marker')
        yield self.actuators.drop_marker()
