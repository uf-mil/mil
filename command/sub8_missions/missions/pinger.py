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
from hydrophones.srv import FindPinger, FindPingerRequest, SetFrequency, SetFrequencyRequest
from hydrophones.msg import ProcessedPing

fprint = text_effects.FprintFactory(title="PINGER", msg_color="cyan").fprint

pub_cam_ray = None

SPEED = 0.1


@util.cancellableInlineCallbacks
def run(sub):

    yield sub.nh.sleep(3)
    sub_position = yield sub.nh.subscribe('/hydrophones/processed', ProcessedPing)
    while True:
        position = yield sub_position.get_next_message()
        position = mil_ros_tools.rosmsg_to_numpy(position.position)
        vec = position / np.linalg.norm(position)
        transform = yield sub._tf_listener.get_transform('base_link', 'hydrophones')
        vec = transform._q_mat.dot(vec)
        vec[2] = 0
        fprint(vec)
        yield sub.move.relative(vec).go(speed=SPEED)
        yield sub.nh.sleep(3)
    
    
