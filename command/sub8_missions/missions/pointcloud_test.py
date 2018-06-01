from txros import util
import numpy as np
from sensor_msgs.msg import CameraInfo
from image_geometry import PinholeCameraModel
from sub8 import SonarPointcloud
from geometry_msgs.msg import Point
import mil_ros_tools
import tf
from twisted.internet import defer


@util.cancellableInlineCallbacks
def run(sub):

    print('Connecting camera')

    cam_info_sub = yield sub.nh.subscribe('/camera/front/left/camera_info',
                                          CameraInfo)

    print('Obtaining cam info message')
    cam_info = yield cam_info_sub.get_next_message()
    model = PinholeCameraModel()
    model.fromCameraInfo(cam_info)

    print('Scanning with blueview')
    spc = SonarPointcloud(sub, [sub.move.zero_roll_and_pitch()])
    x = yield spc.start()
    print(x.shape)

    dice_sub = yield sub.nh.subscribe('/dice/points', Point)

    found = {}
    while len(found) != 2:
        print('Getting dice xy')
        dice_xy = yield dice_sub.get_next_message()
        found[dice_xy.z] = mil_ros_tools.rosmsg_to_numpy(dice_xy)[:2]
        print(found)

    print(found)

    start = sub.move.zero_roll_and_pitch()
    yield start.go()

    dice, xy = found.popitem()
    print('dice {}'.format(dice))
    where = yield find_point(sub, model, xy, x)

    print(where)
    print('YOLO')
    print(sub.pose.position)
    print('zrp')
    yield sub.move.zero_roll_and_pitch().go(blind=True)
    print('hitting')
    yield sub.move.set_position(where).go(blind=True)
    print('going back')
    yield start.go(blind=True)


@util.cancellableInlineCallbacks
def find_point(sub, model, point, x):
    print('Projecting to 3d ray')
    ray = model.projectPixelTo3dRay(point)
    print('Transform')
    transform = yield sub._tf_listener.get_transform('/map', 'front_stereo')
    print('cam_p, cam_q')
    cam_r = tf.transformations.quaternion_matrix(transform._q)[:3, :3]
    ray = cam_r.dot(ray)
    defer.returnValue(find_closest_object_given_ray(x, ray, transform._p))


def find_closest_object_given_ray(points, ray, ray_base, tol=5):
    for p in points:
        distance = np.linalg.norm(np.cross(ray, p - ray_base))
        print(distance)
        if distance > tol:
            continue
        else:
            return p
