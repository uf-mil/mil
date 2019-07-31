from txros import util
import tf
import rospy
import numpy as np
import mil_ros_tools
import visualization_msgs.msg as visualization_msgs
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point, Vector3
from mil_misc_tools import FprintFactory
from .sub_singleton import SubjuGator
from mil_misc_tools import text_effects
from sensor_msgs.msg import CameraInfo, RegionOfInterest
from image_geometry import PinholeCameraModel
from std_srvs.srv import SetBool, SetBoolRequest
from twisted.internet import defer


fprint = text_effects.FprintFactory(title="ARM_TORPDOS", msg_color="cyan").fprint

MISSION = 'Torpedo Challenge'
SPEED = .3
class ArmTorpedos(SubjuGator):

    @util.cancellableInlineCallbacks
    def run(self, args):
        # Assume facing the board        

        global SPEED
            
        fprint('Beginning Torpedo Mission')
        yield self.move.forward

        self.pub_cam_ray = yield self.nh.advertise('/torp/cam_ray', Marker)
        yield self.nh.sleep(1)

        fprint('Connecting camera')

        cam_info_sub = yield self.nh.subscribe('/camera/front/left/camera_info',
                                            CameraInfo)

        fprint('Obtaining cam info message')
        cam_info = yield cam_info_sub.get_next_message()
        model = PinholeCameraModel()
        model.fromCameraInfo(cam_info)

        fprint('Enabling stake/enable')

        enable_service = self.nh.get_service_client("/vision/stake/enable", SetBool)
        yield enable_service(SetBoolRequest(data=True))

        torp_sub = yield self.nh.subscribe('/roi_pub', RegionOfInterest)

        torp_roi = yield torp_sub.get_next_message()
        fprint('Got torp_roi: {}'.format(torp_roi))
        x0 = torp_roi.x_offset
        y0 = torp_roi.y_offset
        height = torp_roi.height
        width = torp_roi.width
        torp_xy = [x0+(width/2), y0+(height/2)]

        ray = yield self.get_transform(model, torp_xy)
        point, norm = self.plane_sonar.get_group_of_points((self.pose.position, ray), min_points=100)
        fprint('point, norm {} {}'.format(point, norm))
        dest = point + norm

        start = self.move.zero_roll_and_pitch()
        yield start.go()

        fprint(dest)
        fprint('Moving!', msg_color='yellow')
        fprint('Current position: {}'.format(self.pose.position))
        fprint('zrp')
        yield self.move.zero_roll_and_pitch().go(blind=True)
        yield self.nh.sleep(4)
  
        fprint('Move in front', msg_color='yellow')
        yield self.move.set_position(dest).go(blind=True, speed=SPEED)
        yield self.nh.sleep(4)
        yield self.move.look_at_without_pitching(point).go(blind=True, speed=SPEED)
        yield self.nh.sleep(1)
        
        fprint('Shooting 1', msg_color='yellow')
        yield self.actuators.shoot_torpedo1()
        self.nh.sleep(1)
        fprint('Shooting 2', msg_color='yellow')
        yield self.actuators.shoot_torpedo2()
        
        fprint('going back', msg_color='yellow')
        yield start.go(blind=True, speed=SPEED)


    @util.cancellableInlineCallbacks
    def get_transform(self, model, point):
        fprint('Projecting to 3d ray')
        ray = np.array(model.projectPixelTo3dRay(point))
        fprint('Transform')
        transform = yield self._tf_listener.get_transform('/map', 'front_left_cam_optical')
        ray = transform._q_mat.dot(ray)
        ray = ray / np.linalg.norm(ray)
        marker = Marker(
            ns='dice',
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
        self.pub_cam_ray.publish(marker)
        fprint('ray: {}, origin: {}'.format(ray, transform._p))
        defer.returnValue(ray)


