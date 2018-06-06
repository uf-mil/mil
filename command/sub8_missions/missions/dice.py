from txros import util
import numpy as np
from sensor_msgs.msg import CameraInfo
from image_geometry import PinholeCameraModel
from sub8 import SonarObjects
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
        print("No objects")
        defer.returnValue(False)

    where = mil_ros_tools.rosmsg_to_numpy(objs.objects[0].pose.position)

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
def get_transform(sub, model, point):
    print('Projecting to 3d ray')
    ray = model.projectPixelTo3dRay(point)
    print('Transform')
    transform = yield sub._tf_listener.get_transform('/map', 'front_left_cam')
    cam_r = tf.transformations.quaternion_matrix(transform._q)[:3, :3]
    ray = cam_r.dot(ray)
    ray = ray/np.linalg.norm(ray)
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
    defer.returnValue((ray, transform._p))
