from .sub_singleton import SubjuGator, Searcher, VisionProxy
from txros import util
import numpy as np
from sensor_msgs.msg import CameraInfo
from image_geometry import PinholeCameraModel
import mil_ros_tools
from twisted.internet import defer
from geometry_msgs.msg import Point
from mil_misc_tools import text_effects
from std_srvs.srv import SetBool, SetBoolRequest

from sub8_msgs.srv import VisionRequest, VisionRequestRequest, VisionRequest2DRequest, VisionRequest2D

import mil_ros_tools
from std_srvs.srv import Trigger
import math

import genpy

fprint = text_effects.FprintFactory(title="DRACULA_GRAB", msg_color="cyan").fprint

SPEED = 0.25
FAST_SPEED = 1

SEARCH_DEPTH= 1.5
DEPTH_DRACULA_GRABBER = 2.1
TRAVEL_DEPTH = 1.6  # 2
SEARCH_POINTS = 8
SEARCH_RADII = [0.75,1.5]



class DraculaGrabber(SubjuGator):

    @util.cancellableInlineCallbacks
    def run(self, args):

        start_position = self.pose.position
        fprint('Enabling cam_ray publisher')

        yield self.nh.sleep(1)

        fprint('Connecting camera')


        enable_service = self.nh.get_service_client("/vision/dracula/enable", SetBool)

        yield enable_service(SetBoolRequest(data=True))
        fprint('Obtaining cam info message')
        cam_info = yield self.down_cam_info.get_next_message()
        cam_center = np.array([cam_info.width/2, cam_info.height/2])
        cam_norm = np.sqrt(cam_center[0]**2 + cam_center[1]**2)
        fprint('Cam center: {}'.format(cam_center))

        model = PinholeCameraModel()
        model.fromCameraInfo(cam_info)

        yield self.move.depth(SEARCH_DEPTH).zero_roll_and_pitch().go(speed=SPEED)
        '''
        self.vision_proxies.vampire_identifier.start()
        pattern = gen_pattern(SEARCH_POINTS, SEARCH_RADII)
        search = Searcher(self, self.vision_proxies.vampire_identifier.get_2d, pattern)
        yield search.start_search(timeout = 240)
        '''

        fprint("Starting Pattern")
        flag = True
        count = 0
        pattern = gen_pattern(SEARCH_POINTS, SEARCH_RADII)
        while(flag and count < 24):
          fprint(count)
          dracula_msg = self.bbox_sub.get_next_message().addErrback(lambda x: None)

          start_time = yield self.nh.get_time()
          while self.nh.get_time() - start_time < genpy.Duration(2): 
            if len(dracula_msg.callbacks) == 0:
              fprint('Time out, move again')
              dracula_msg.cancel()

            yield self.nh.sleep(0.5)
            dracula_msg.cancel()
          if count == 24:
             defer.returnValue(False)
          x = yield dracula_msg
          fprint(x)
          if x is not None:
            flag = False
            break
          i = count % len(pattern)
          print pattern[i]
          yield self.move.relative(pattern[i]).depth(TRAVEL_DEPTH).go(speed=SPEED)
          # Let controller catch up a bit
          yield self.nh.sleep(0.5)
          count = count + 1

        fprint('FOUND! Trying to center')


        for i in range(20):
            fprint('Getting location of dracula...')
            res = yield self.bbox_sub.get_next_message()
        
            dracula_xy = np.array([res.x, res.y])
            vec = dracula_xy - cam_center
            vec2 = [-vec[1], -vec[0]]

            if np.linalg.norm(vec) < 50:
                break
            fprint("Vec: {}".format(vec2))
            vec2 = vec2 / cam_norm

            fprint("Rel move vec {}".format(vec2))
            vec2 = np.append(vec2, 0)

            yield self.move.relative_depth(vec2).go(speed=SPEED)

        yield self.actuators.gripper_open()
        # self.vision_proxies.vampire_identifier.stop()
        fprint('Centered, going to depth {}'.format(DEPTH_DRACULA_GRABBER))
        yield self.move.depth(DEPTH_DRACULA_GRABBER).backward(.2).zero_roll_and_pitch().go(speed=SPEED)
        yield self.nh.sleep(3)
        fprint('Dropping marker')
        yield self.actuators.gripper_close()
        #yield self.move.set_position(start_position).go(speed=0.3)
        #yield self.move.depth(0.2).go()
        #yield self.nh.sleep(5)
        #yield self.move.depth(1.5).go()
        #yield self.actuators.gripper_open()
        


def gen_pattern(steps, radii=[]):
    assert(steps > 0)
    unit_pattern = [np.array([math.cos(i*(math.pi*2/steps)),
                         math.sin(i*(math.pi*2/steps)), 0])
                         for i in range(steps)]
    pattern = [np.array([0,0,0])]
    for r in radii:
        for unit_v in unit_pattern:
            pattern.append(unit_v*r)
    rel_pattern = [np.array([0,0,0])]
    for idx, point in enumerate(pattern):
        if idx > 0:
            rel_pattern.append(point - pattern[idx-1])
    rel_pattern.append(-pattern[-1])
    return rel_pattern
