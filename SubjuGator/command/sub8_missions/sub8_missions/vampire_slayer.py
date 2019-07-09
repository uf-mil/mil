from .sub_singleton import SubjuGator
from txros import util
import numpy as np
from sensor_msgs.msg import CameraInfo
from image_geometry import PinholeCameraModel
import mil_ros_tools
from twisted.internet import defer
import visualization_msgs.msg as visualization_msgs
from geometry_msgs.msg import Point, Vector3
from visualization_msgs.msg import Marker
from mil_misc_tools import text_effects
from std_srvs.srv import SetBool, SetBoolRequest

from sub8_msgs.srv import GuessRequest, GuessRequestRequest

import mil_ros_tools

fprint = text_effects.FprintFactory(title="VAMPIRES", msg_color="cyan").fprint

pub_cam_ray = None

SPEED = 0.25


class VampireSlayer(SubjuGator):

    @util.cancellableInlineCallbacks
    def run(self, args):
        global SPEED
        global pub_cam_ray
        fprint('Enabling cam_ray publisher')
        pub_cam_ray = yield self.nh.advertise('/vamp/cam_ray', Marker)

        yield self.nh.sleep(1)

        fprint('Connecting camera')

        cam_info_sub = yield self.nh.subscribe(
            '/camera/front/left/camera_info',
                                              CameraInfo)

        fprint('Obtaining cam info message')
        cam_info = yield cam_info_sub.get_next_message()
        model = PinholeCameraModel()
        model.fromCameraInfo(cam_info)

     #   enable_service = self.nh.get_service_client("/vamp/enable", SetBool)
     #   yield enable_service(SetBoolRequest(data=True))

        try:
            vamp_txros = yield self.nh.get_service_client('/guess_location',
                                                          GuessRequest)
            vamp_req = yield vamp_txros(GuessRequestRequest(item='vampire_slayer'))
            if vamp_req.found is False:
                use_prediction = False
                fprint(
                    'Forgot to add vampires to guess?',
                    msg_color='yellow')
            else:
                fprint('Found vamires.', msg_color='green')
                yield self.move.set_position(mil_ros_tools.rosmsg_to_numpy(vamp_req.location.pose.position)).depth(0.5).go(speed=0.4)
        except Exception as e:
            fprint(e)

        dice_sub = yield self.nh.subscribe('/bbox_pub', Point)

        found = {}
        history_tf = {}
        while len(found) != 1:
            fprint('Getting target vampire xy')
            dice_xy = yield dice_sub.get_next_message()
            found[dice_xy.z] = mil_ros_tools.rosmsg_to_numpy(dice_xy)[:2]
            fprint(found)
            out = yield self.get_transform(model, found[dice_xy.z])
            history_tf[dice_xy.z] = out

        # yield enable_service(SetBoolRequest(data=False))

        start = self.move.zero_roll_and_pitch()
        yield start.go()

        for i in range(1):
            fprint('Hitting Vampire {}'.format(i))
            # Get one of the dice
            dice, xy = found.popitem()
            fprint('Vampire: {}'.format(dice))
            ray, base = history_tf[dice]

            where = base + 3 * ray

            fprint(where)
            fprint('Moving!', msg_color='yellow')
            fprint('Current position: {}'.format(self.pose.position))
            fprint('zrp')
            yield self.move.zero_roll_and_pitch().go(blind=True)
            yield self.nh.sleep(4)
            fprint('hitting', msg_color='yellow')
            yield self.move.look_at(where).go(blind=True, speed=SPEED)
            yield self.nh.sleep(4)
            yield self.move.set_position(where).go(blind=True, speed=SPEED)
            yield self.nh.sleep(4)
            fprint('going back', msg_color='yellow')
            yield start.go(blind=True, speed=SPEED)
            yield self.nh.sleep(4)

    @util.cancellableInlineCallbacks
    def get_transform(self, model, point):
        fprint('Projecting to 3d ray')
        ray = np.array(model.projectPixelTo3dRay(point))
        fprint("ddray {}".format(ray))
        fprint('Transform')
        transform = yield self._tf_listener.get_transform('/map', 'front_left_cam_optical')
        ray = transform._q_mat.dot(ray)
        # ray = ray / np.linalg.norm(ray)
        marker = Marker(
            ns='vamp',
            action=visualization_msgs.Marker.ADD,
            type=Marker.ARROW,
            scale=Vector3(0.2, 0.2, 2),
            points=np.array([
                Point(transform._p[0], transform._p[1], transform._p[2]),
                Point(transform._p[0] + ray[0], transform._p[1] + ray[1],
                      transform._p[2] + ray[2]),
            ]))
        marker.header.frame_id = '/map'
        marker.color.r = 0
        marker.color.g = 1
        marker.color.b = 0
        marker.color.a = 1
        global pub_cam_ray
        pub_cam_ray.publish(marker)
        fprint('ray: {}, origin: {}'.format(ray, transform._p))
        defer.returnValue((ray, transform._p))
