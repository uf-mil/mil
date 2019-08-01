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

fprint = text_effects.FprintFactory(title="DRACULA_GRAB", msg_color="cyan").fprint

SPEED = 0.25
FAST_SPEED = 1

SEARCH_HEIGHT = 1.5
HEIGHT_DRACULA_GRABBER = 0.5
TRAVEL_DEPTH = 0.5  # 2


class DraculaGrabber(SubjuGator):

    @util.cancellableInlineCallbacks
    def run(self, args):
        fprint('Enabling cam_ray publisher')

        yield self.nh.sleep(1)

        fprint('Connecting camera')

        cam_info_sub = yield self.nh.subscribe(
            '/camera/down/camera_info',
            CameraInfo)

        fprint('Obtaining cam info message')
        cam_info = yield cam_info_sub.get_next_message()
        cam_center = np.array([cam_info.width/2, cam_info.height/2])
        cam_norm = np.sqrt(cam_center[0]**2 + cam_center[1]**2)
        fprint('Cam center: {}'.format(cam_center))

        model = PinholeCameraModel()
        model.fromCameraInfo(cam_info)

     #   enable_service = self.nh.get_service_client("/vamp/enable", SetBool)
     #   yield enable_service(SetBoolRequest(data=True))

        dracula_sub = yield self.nh.subscribe('/yellow_vectors', Point)
        yield self.move.to_height(SEARCH_HEIGHT).zero_roll_and_pitch().go(speed=SPEED)
        i = 0
        while i < 20:
            fprint('Getting location of dracula...')
            dracula_msg = yield dracula_sub.get_next_message()
            dracula_xy = mil_ros_tools.rosmsg_to_numpy(dracula_msg)[:2]
            vec = dracula_xy - cam_center
            vec2 = [-vec[1], -vec[0]]

            if np.linalg.norm(vec) < 50:
                break
            fprint("Vec: {}".format(vec2))
            vec2 = vec2 / cam_norm

            fprint("Rel move vec {}".format(vec2))
            vec2 = np.append(vec2, 0)

            yield self.move.relative_depth(vec2).go(speed=SPEED)
            i+=1

        fprint('Centered, going to depth {}'.format(HEIGHT_DRACULA_GRABBER))
        yield self.move.to_height(HEIGHT_DRACULA_GRABBER).zero_roll_and_pitch().go(speed=SPEED)
        fprint('Dropping marker')
        yield self.actuators.gripper_close()
        
