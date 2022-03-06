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
from mil_misc_tools import text_effects, PrintColors
from std_srvs.srv import SetBool, SetBoolRequest
from visualization_msgs.msg import Marker, MarkerArray
from mil_misc_tools import FprintFactory
from mil_ros_tools import rosmsg_to_numpy

from sub8_msgs.srv import GuessRequest, GuessRequestRequest

import mil_ros_tools

fprint = text_effects.FprintFactory(title="VAMPIRES", msg_color=PrintColors.CYAN).fprint

pub_cam_ray = None

SPEED = 0.25

X_OFFSET = 0
Y_OFFSET = 0
Z_OFFSET = 0
class VampireSlayer(SubjuGator):

    @util.cancellableInlineCallbacks
    def run(self, args):
        self.vision_proxies.xyz_points.start()

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

#        try:
#            vamp_txros = yield self.nh.get_service_client('/guess_location',
#                                                          GuessRequest)
#            vamp_req = yield vamp_txros(GuessRequestRequest(item='vampire_slayer'))
#            if vamp_req.found is False:
#                use_prediction = False
#                fprint(
#                    'Forgot to add vampires to guess?',
#                    msg_color='yellow')
#            else:
#                fprint('Found vampires.', msg_color='green')
#                yield self.move.set_position(mil_ros_tools.rosmsg_to_numpy(vamp_req.location.pose.position)).depth(0.5).go(speed=0.4)
#        except Exception as e:
#            fprint(e)

        markers = MarkerArray()
        pub_markers = yield self.nh.advertise('/torpedo/rays', MarkerArray)
        pub_markers.publish(markers)
        '''
        Move Pattern
        '''
        yield self.move.left(1).go()
        yield self.nh.sleep(2)
        yield self.move.right(2).go()
        yield self.nh.sleep(2)
        yield self.move.down(.5).go()
        yield self.nh.sleep(2)
        yield self.move.up(.5).go()
        yield self.move.forward(.75).go()
        yield self.nh.sleep(2)
        yield self.move.right(.75).go()
        yield self.nh.sleep(2)
        yield self.move.backward(.75).go()
        yield self.move.left(1).go()
        '''
        Did we find something?
        '''
        res = yield self.vision_proxies.xyz_points.get_pose(
            target='buoy')
        MISSION = 'Vampires'
        print_info = FprintFactory(title=MISSION).fprint
        
        if res.found:
            print_info("CHARGING BUOY")
            target_pose = rosmsg_to_numpy(res.pose.pose.position)
            target_normal = rosmsg_to_numpy(res.pose.pose.orientation)[:2]
            print('Normal: ', target_normal)
            yield self.move.go(blind=True, speed=0.1)  # Station hold
            transform = yield self._tf_listener.get_transform('/map', '/base_link')
            target_position = target_pose
#            target_position = target_pose / target_normal
            
            

            sub_pos = yield self.tx_pose()
            print('Current Sub Position: ', sub_pos)

            print('Map Position: ', target_position)
            #sub_pos = transform._q_mat.dot(sub_pos[0] - transform._p)
            #target_position = target_position - sub_pos[0]
            # yield self.move.look_at_without_pitching(target_position).go(blind=True, speed=.25)
            #yield self.move.relative(np.array([0, target_position[1], 0])).go(blind=True, speed=.1)
            # Don't hit buoy yet
            print("MOVING TO X: ", target_position[0])
            print("MOVING TO Y: ", target_position[1])
            yield self.move.set_position(np.array([target_position[0], target_position[1], target_position[2]])).go(
                blind=True, speed=.1)
            # Go behind it
            #print('Going behind target')
            #yield self.move.right(4).go(speed=1)
            #yield self.move.forward(4).go(speed=1)
            #yield self.move.left(4).go(speed=1)
            # Hit buoy
            #print('Hitting Target')
            #yield self.move.strafe_backward(Y_OFFSET).go(speed=1)
            print_info(
                "Slaying the Vampire, good job Inquisitor.")
            sub_pos = yield self.tx_pose()
            print('Current Sub Position: ', sub_pos)
            marker = Marker(
                ns='buoy',
                action=visualization_msgs.Marker.ADD,
                type=Marker.ARROW,
                scale=Vector3(0.2, 0.5, 0),
                points=np.array([Point(0, 0, 0),
                                    res.pose.pose.position]))
            marker.id = 3
            marker.header.frame_id = '/base_link'
            marker.color.r = 1
            marker.color.g = 0
            marker.color.a = 1
            markers.markers.append(marker)
            pub_markers.publish(markers)

            yield self.nh.sleep(0.5)  # Throttle service calls
            # print_info(info)
            self.vision_proxies.xyz_points.stop()
