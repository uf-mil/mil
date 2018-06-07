from txros import util
import numpy as np
from sensor_msgs.msg import CameraInfo
from image_geometry import PinholeCameraModel
from sub8 import SonarObjects
import mil_ros_tools
from twisted.internet import defer
import visualization_msgs.msg as visualization_msgs
from geometry_msgs.msg import Point, Vector3
from visualization_msgs.msg import Marker
from mil_misc_tools import text_effects

fprint = text_effects.FprintFactory(title="DICE", msg_color="cyan").fprint

pub_cam_ray = None

SPEED = 0.3


@util.cancellableInlineCallbacks
def run(sub):
    global SPEED
    global pub_cam_ray
    fprint('Enabling cam_ray publisher')
    pub_cam_ray = yield sub.nh.advertise('/dice/cam_ray', Marker)

    yield sub.nh.sleep(1)

    fprint('Connecting camera')

    cam_info_sub = yield sub.nh.subscribe('/camera/front/left/camera_info',
                                          CameraInfo)

    fprint('Obtaining cam info message')
    cam_info = yield cam_info_sub.get_next_message()
    model = PinholeCameraModel()
    model.fromCameraInfo(cam_info)

    dice_sub = yield sub.nh.subscribe('/dice/points', Point)

    found = {}
    while len(found) != 1:
        fprint('Getting dice xy')
        dice_xy = yield dice_sub.get_next_message()
        found[dice_xy.z] = mil_ros_tools.rosmsg_to_numpy(dice_xy)[:2]
        fprint(found)

    start = sub.move.zero_roll_and_pitch()
    yield start.go()

    # Get one of the dice
    dice, xy = found.popitem()
    fprint('dice {}'.format(dice))
    ray, base = yield get_transform(sub, model, xy)

    so = SonarObjects(sub, [
        start.left(0.5),
        start.right(1),
        start.left(0.5),
        start.pitch_down_deg(15), start
    ])
    objs = yield so.start_until_found_in_cone(
        base,
        clear=False,
        object_count=1,
        ray=ray,
        angle_tol=20,
        distance_tol=13)
    if objs is None:
        fprint("No objects")
        defer.returnValue(False)

    where = mil_ros_tools.rosmsg_to_numpy(objs.objects[0].pose.position)

    if where is None:
        fprint("Did not find anything")
        defer.returnValue(False)

    fprint(where)
    fprint('Moving!', msg_color='yellow')
    fprint('Current position: {}'.format(sub.pose.position))
    fprint('zrp')
    yield sub.move.zero_roll_and_pitch().go(blind=True)
    fprint('hitting', msg_color='yellow')
    yield sub.move.look_at(where).go(blind=True, speed=SPEED)
    yield sub.move.set_position(where).go(blind=True, speed=SPEED)
    fprint('going back', msg_color='yellow')
    yield start.go(blind=True, speed=SPEED)


@util.cancellableInlineCallbacks
def get_transform(sub, model, point):
    fprint('Projecting to 3d ray')
    ray = np.array(model.projectPixelTo3dRay(point))
    fprint('Transform')
    transform = yield sub._tf_listener.get_transform('/map', 'front_left_cam')
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
    defer.returnValue((ray, transform._p))
