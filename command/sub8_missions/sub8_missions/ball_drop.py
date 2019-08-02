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

import genpy

fprint = text_effects.FprintFactory(title="BALL_DROP", msg_color="cyan").fprint

SPEED = 0.25
FAST_SPEED = 1

SEARCH_HEIGHT = 3
HEIGHT_BALL_DROPER = 2.3
TRAVEL_DEPTH = 1  # 2
SEARCH_POINTS = 4
SEARCH_RADII = [i for i in range(1,5)]

class BallDrop(SubjuGator):

    @util.cancellableInlineCallbacks
    def run(self, args):
        enable_service = self.nh.get_service_client("/vision/garlic/enable", SetBool)
        yield enable_service(SetBoolRequest(data=True))
        
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
    
        try:
            position = yield self.poi.get('ball_drop')
            fprint('Found ball_drop: {}'.format(position), msg_color='green')
            yield self.move.look_at_without_pitching(position).set_position(position).depth(TRAVEL_DEPTH).go(speed=FAST_SPEED)
        except Exception as e:
            fprint(str(e) + 'Forgot to run guess server?', msg_color='yellow')

        ball_drop_sub = yield self.nh.subscribe('/bbox_pub', Point)
        yield self.move.to_height(SEARCH_HEIGHT).zero_roll_and_pitch().go(speed=SPEED)

        fprint("Starting Pattern")
        flag = True
        count = 0
        pattern = gen_pattern(SEARCH_POINTS, RADII)
        while(flag and count < 24):
          fprint(count)
          ball_drop_msg = ball_drop_sub.get_next_message().addErrback(lambda x: None)

          start_time = yield self.nh.get_time()
          while self.nh.get_time() - start_time < genpy.Duration(2): 
            if len(ball_drop_msg.callbacks) == 0:
              fprint('Time out, move again')
              ball_drop_msg.cancel()
              #flag = False
            yield self.nh.sleep(0.5)
            ball_drop_msg.cancel()
          if count == 24:
             defer.returnValue(False)
          x = yield ball_drop_sub
          fprint(x)
          if x is not None:
            flag = False
          i = count % len(pattern)
          yield self.move.relative(pattern[i]).go(speed=SPEED)
          count = count + 1

        fprint('FOUND! Trying to center')

        for i in range(20):
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

        yield enable_service(SetBoolRequest(data=False))
        fprint('Centered, going to depth {}'.format(HEIGHT_BALL_DROPER))
        yield self.move.to_height(HEIGHT_BALL_DROPER).zero_roll_and_pitch().go(speed=SPEED)
        fprint('Dropping marker')
        yield self.actuators.drop_marker()


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
        print idx
        if idx > 0:
            rel_pattern.append(point - pattern[idx-1])
    rel_pattern.append(-pattern[-1])
    return rel_pattern
