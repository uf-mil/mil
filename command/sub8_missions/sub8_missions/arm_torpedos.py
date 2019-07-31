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

MISSION = 'Torpedo Challenge'
SPEED = .75
class ArmTorpedos(SubjuGator):

    @util.cancellableInlineCallbacks
    def run(self):
        global SPEED
        yaw = tf.transformations.euler_from_quaternion(rot, 'syxz')[0]
        if abs(yaw) > 180:
            yield self.move.look_at([0, 0, 0]).go()
            yield self.sleep(2)
            yield self.move.forward(1).go()
            yield self.sleep(2)
            yield self.move.set_depth(1.8).go()
            yield self.sleep(2)
            yield self.move.yaw_right(160).go()
        else:
            yield self.sleep(2)
            yield self.move.back(1).go()
            yield self.sleep(2)
            yield self.move.set_depth(1.8).go()
    
        fprint('Beginning Torpedo Mission')
        yield self.move.forward

        pub_cam_ray = yield self.nh.advertise('/torp/cam_ray', Marker)

        yield self.nh.sleep(1)

        fprint('Connecting camera')

        cam_info_sub = yield self.nh.subscribe('/camera/front/left/camera_info',
                                            CameraInfo)

        fprint('Obtaining cam info message')
        cam_info = yield cam_info_sub.get_next_message()
        model = PinholeCameraModel()
        model.fromCameraInfo(cam_info)

        enable_service = self.nh.get_service_client("/stake/enable", SetBool)
        yield enable_service(SetBoolRequest(data=True))

        torp_sub = yield self.nh.subscribe('/roi_pub', RegionOfInterest)

        torp_roi = yield torp_sub.get_next_message()
        x0 = roi.x_offset
        y0 = roi.y_offset
        height = roi.height
        width = roi.width
        torp_xy = [x0+(width/2), y0+(height/2)]

        out = yield get_transform(sub, model, torp_xy)
        point, norm = self.plane_sonar.get_group_of_points((self.pose.position, ray))
        dest = point + norm

        start = sub.move.zero_roll_and_pitch()

        yield start.go()

        fprint(dest)
        fprint('Moving!', msg_color='yellow')
        fprint('Current position: {}'.format(sub.pose.position))
        fprint('zrp')
        yield sub.move.zero_roll_and_pitch().go(blind=True)
        yield self.nh.sleep(4)
        fprint('hitting', msg_color='yellow')
        yield sub.move.look_at(dest).go(blind=True, speed=SPEED)
        yield self.nh.sleep(4)
        yield sub.move.set_position(dest).go(blind=True, speed=SPEED)
        yield self.nh.sleep(4)
        fprint('going back', msg_color='yellow')
        yield start.go(blind=True, speed=SPEED)
        yield self.nh.sleep(4)


    @util.cancellableInlineCallbacks
    def get_transform(sub, model, point):
        fprint('Projecting to 3d ray')
        ray = np.array(model.projectPixelTo3dRay(point))
        fprint('Transform')
        transform = yield sub._tf_listener.get_transform('/map', 'front_left_cam_optical')
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
        global pub_cam_ray
        pub_cam_ray.publish(marker)
        fprint('ray: {}, origin: {}'.format(ray, transform._p))
        defer.returnValue(ray)


