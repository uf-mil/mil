#!/usr/bin/env python
from geometry_msgs.msg import Pose, Point, PoseStamped, Vector3
from mil_misc_tools import text_effects
from mil_msgs.srv import SetGeometry
import numpy as np
import rospy
from std_srvs.srv import SetBool, SetBoolResponse
from sub8_msgs.srv import VisionRequestResponse, VisionRequest, VisionRequest2D
from gazebo_msgs.srv import GetModelState
from std_msgs.msg import Header
import os
import rospkg
import sys
import yaml


rospack = rospkg.RosPack()
config_file = os.path.join(rospack.get_path(
    'sub8_missions'), 'sub8', 'vision_proxies.yaml')
f = yaml.load(open(config_file, 'r'))
model_state = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)

fprint = text_effects.FprintFactory(title="SIMULATOR").fprint


def handle_fake_perception(extra, target_object):
    '''
    Calls the GetModelState service from gazebo to get the realtime position of the model targeted.
    Provides this information to the mission.
    @param extra The target_name passed through some missions.
    Other missions do not pass a target_name thus its lable of extra.
    @param target_object Is the model name of the object targeted by the mission.
    Missions that do not pass a target_name must use this.
    '''

    now = rospy.get_rostime()
    k = np.uint32(0)
    if extra != '':
        target_object = extra
    if target_object == '':
        fprint("NO TARGET")
        sys.exit(0)
    model = get_position(target_object)
    # Library of offets. Models must be manually offset as gazebo coordinates != center of model.
    centlib = {'start_gate': Point(1.5, 0, 0), 'nav_gate': Point(1.15, 0, 0)}
    if target_object in centlib:
        offset = centlib[target_object]
    else:
        offset = Point(0, 0, 0)
    pose_stamp = PoseStamped(header=Header(seq=k, stamp=now, frame_id="/map"),
                             # Offset our pose by the starting position of the sub relative to the world in Gazebo.
                             pose=Pose(position=Point(model.pose.position.x - 13 + offset.x,
                                                      model.pose.position.y - 24 + offset.y, - 1 + offset.z),
                                       orientation=model.pose.orientation))
    covariance_diagonal = Vector3(0, 0, 0)
    found = True
    resp2 = VisionRequestResponse(pose_stamp, covariance_diagonal, found)

    return resp2


def get_position(model_name):

    try:
        resp1 = model_state(model_name, 'world')
        return resp1
    except rospy.ServiceException:
        print None


def set_geometry(req):

    return {'success': True}


def vision_cb_2D():

    return False


def start(resp):

    return SetBoolResponse(True, "")


def init_service(name, target):

    # Generates services required for missions and target aquisition
    rospy.Service('/vision/' + name + '/pose', VisionRequest,
                  lambda h: handle_fake_perception(h.target_name, target))
    # The following three services do nothing other than return true values.
    # They are not needed in sim but a return value is required for missions.
    rospy.Service('/vision/' + name + '/set_geometry',
                  SetGeometry, set_geometry)
    rospy.Service('/vision/' + name + '/2D', VisionRequest2D, vision_cb_2D)
    rospy.Service('/vision/' + name + '/enable', SetBool, start)


def fake_perception_server():

    rospy.init_node('fake_perception')
    '''
    In the dictionary below please place the name of the service you wish to mimic and the target of said service.
    The target should match the model name found in the duck.launch file. It will be what immediately follows the
    -model tag within the node tied to the model. Example is orange_rectangle mapped to channel_marker_1.
    If the service provides a target_name you may leave the target empty as done with buoys.
    '''
    missions = {'orange_rectangle': 'channel_marker_1',
                'buoys': '', 'start_gate': 'start_gate'}
    for key in missions:
        init_service(key, missions[key])

    fprint("Faking perception.")
    rospy.spin()


if __name__ == "__main__":

    fake_perception_server()
    rospy.wait_for_service('/gazebo/get_model_state')
