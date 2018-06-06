from txros import util
import numpy as np
from sensor_msgs.msg import CameraInfo
from image_geometry import PinholeCameraModel
from sub8 import SonarPointcloud
from geometry_msgs.msg import Point
import mil_ros_tools
import tf
from twisted.internet import defer
import visualization_msgs.msg as visualization_msgs
from geometry_msgs.msg import Point, Vector3
from visualization_msgs.msg import Marker

pub_cam_ray = None


@util.cancellableInlineCallbacks
def run(sub):

    print('Enabling cam_ray publisher')
    global pub_cam_ray
    pub_cam_ray = yield sub.nh.advertise('/pointcloud_test/cam_ray', Marker)

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
    if where is None:
        print("Did not find anything")
        defer.returnValue(False)

    print(where)
    print('YOLO')
    print(sub.pose.position)
    print('zrp')
    yield sub.move.zero_roll_and_pitch().go(blind=True)
    print('hitting')
    yield sub.move.look_at(where).go(blind=True)
    yield sub.move.set_position(where).go(blind=True)
    print('going back')
    yield start.go(blind=True)


@util.cancellableInlineCallbacks
def find_point(sub, model, point, x):
    print('Projecting to 3d ray')
    ray = model.projectPixelTo3dRay(point)
    print('Transform')
    transform = yield sub._tf_listener.get_transform('/map', 'front_stereo')
    cam_r = tf.transformations.quaternion_matrix(transform._q)[:3, :3]
    ray = cam_r.dot(ray)
    marker = Marker(
        ns='pointcloud',
        action=visualization_msgs.Marker.ADD,
        type=Marker.ARROW,
        scale=Vector3(0.2, 0.2, 2),
        points=np.array([
            Point(transform._p[0], transform._p[1], transform._p[2]),
            Point(ray[0], ray[1], ray[2])
        ]))
    marker.header.frame_id = '/map'
    marker.color.r = 0
    marker.color.g = 1
    marker.color.b = 0
    marker.color.a = 1
    global pub_cam_ray
    pub_cam_ray.publish(marker)
    print('ray: {}, origin: {}'.format(ray, transform._p))
    defer.returnValue(find_closest_object_given_ray(x, ray, transform._p))


def find_closest_object_given_ray(points,
                                  ray,
                                  ray_base,
                                  tol_min=1,
                                  tol_max=5,
                                  angle_deg_tol=30):
    ray_norm = np.linalg.norm(ray)
    angle_rad_tol = angle_deg_tol * np.pi / 180
    all_dots = np.apply_along_axis(lambda v: np.arccos(ray.dot(v - ray_base)/(ray_norm*np.linalg.norm(v - ray_base))), 1, points)
    index_sort_angle = np.argsort(all_dots)
    min_dist = tol_max
    ret_p = None
    for idx in index_sort_angle:
        p = points[idx]
        if all_dots[idx] > angle_rad_tol:
            continue
        distance = np.linalg.norm(np.cross(ray, p - ray_base))
        if distance < tol_max and distance > tol_min and distance < min_dist:
            min_dist = distance
            ret_p = p
    return ret_p
