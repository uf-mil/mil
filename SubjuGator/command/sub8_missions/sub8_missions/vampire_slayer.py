import mil_ros_tools
import numpy as np
import visualization_msgs.msg as visualization_msgs
from geometry_msgs.msg import Point, Vector3
from image_geometry import PinholeCameraModel
from mil_misc_tools import FprintFactory, text_effects
from mil_ros_tools import rosmsg_to_numpy
from sensor_msgs.msg import CameraInfo
from std_srvs.srv import SetBool, SetBoolRequest
from sub8_msgs.srv import GuessRequest, GuessRequestRequest
from twisted.internet import defer
from txros import util
from visualization_msgs.msg import Marker, MarkerArray

from .sub_singleton import SubjuGator

fprint = text_effects.FprintFactory(title="VAMPIRES", msg_color="cyan").fprint

pub_cam_ray = None

SPEED = 0.25

X_OFFSET = 0
Y_OFFSET = 0
Z_OFFSET = 0


class VampireSlayer(SubjuGator):
    async def run(self, args):
        await self.vision_proxies.xyz_points.start()

        global SPEED
        global pub_cam_ray
        fprint("Enabling cam_ray publisher")
        pub_cam_ray = self.nh.advertise("/vamp/cam_ray", Marker)
        await pub_cam_ray.setup()

        await self.nh.sleep(1)

        fprint("Connecting camera")

        cam_info_sub = self.nh.subscribe("/camera/front/left/camera_info", CameraInfo)
        await cam_info_sub.setup()

        fprint("Obtaining cam info message")
        cam_info = await cam_info_sub.get_next_message()
        model = PinholeCameraModel()
        model.fromCameraInfo(cam_info)

        #   enable_service = self.nh.get_service_client("/vamp/enable", SetBool)
        #   await enable_service(SetBoolRequest(data=True))

        #        try:
        #            vamp_txros = await self.nh.get_service_client('/guess_location',
        #                                                          GuessRequest)
        #            vamp_req = await vamp_txros(GuessRequestRequest(item='vampire_slayer'))
        #            if vamp_req.found is False:
        #                use_prediction = False
        #                fprint(
        #                    'Forgot to add vampires to guess?',
        #                    msg_color='yellow')
        #            else:
        #                fprint('Found vampires.', msg_color='green')
        #                await self.move.set_position(mil_ros_tools.rosmsg_to_numpy(vamp_req.location.pose.position)).depth(0.5).go(speed=0.4)
        #        except Exception as e:
        #            fprint(e)

        markers = MarkerArray()
        pub_markers = self.nh.advertise("/torpedo/rays", MarkerArray)
        await pub_markers.setup()
        pub_markers.publish(markers)
        """
        Move Pattern
        """
        await self.move.left(1).go()
        await self.nh.sleep(2)
        await self.move.right(2).go()
        await self.nh.sleep(2)
        await self.move.down(0.5).go()
        await self.nh.sleep(2)
        await self.move.up(0.5).go()
        await self.move.forward(0.75).go()
        await self.nh.sleep(2)
        await self.move.right(0.75).go()
        await self.nh.sleep(2)
        await self.move.backward(0.75).go()
        await self.move.left(1).go()
        """
        Did we find something?
        """
        res = await self.vision_proxies.xyz_points.get_pose(target="buoy")
        MISSION = "Vampires"
        print_info = FprintFactory(title=MISSION).fprint

        if res.found:
            print_info("CHARGING BUOY")
            target_pose = rosmsg_to_numpy(res.pose.pose.position)
            target_normal = rosmsg_to_numpy(res.pose.pose.orientation)[:2]
            print("Normal: ", target_normal)
            await self.move.go(blind=True, speed=0.1)  # Station hold
            transform = await self._tf_listener.get_transform("map", "/base_link")
            target_position = target_pose
            #            target_position = target_pose / target_normal

            sub_pos = await self.tx_pose()
            print("Current Sub Position: ", sub_pos)

            print("Map Position: ", target_position)
            # sub_pos = transform._q_mat.dot(sub_pos[0] - transform._p)
            # target_position = target_position - sub_pos[0]
            # await self.move.look_at_without_pitching(target_position).go(blind=True, speed=.25)
            # await self.move.relative(np.array([0, target_position[1], 0])).go(blind=True, speed=.1)
            # Don't hit buoy yet
            print("MOVING TO X: ", target_position[0])
            print("MOVING TO Y: ", target_position[1])
            await self.move.set_position(
                np.array([target_position[0], target_position[1], target_position[2]])
            ).go(blind=True, speed=0.1)
            # Go behind it
            # print('Going behind target')
            # await self.move.right(4).go(speed=1)
            # await self.move.forward(4).go(speed=1)
            # await self.move.left(4).go(speed=1)
            # Hit buoy
            # print('Hitting Target')
            # await self.move.strafe_backward(Y_OFFSET).go(speed=1)
            print_info("Slaying the Vampire, good job Inquisitor.")
            sub_pos = await self.tx_pose()
            print("Current Sub Position: ", sub_pos)
            marker = Marker(
                ns="buoy",
                action=visualization_msgs.Marker.ADD,
                type=Marker.ARROW,
                scale=Vector3(0.2, 0.5, 0),
                points=np.array([Point(0, 0, 0), res.pose.pose.position]),
            )
            marker.id = 3
            marker.header.frame_id = "/base_link"
            marker.color.r = 1
            marker.color.g = 0
            marker.color.a = 1
            markers.markers.append(marker)
            pub_markers.publish(markers)

            await self.nh.sleep(0.5)  # Throttle service calls
            # print_info(info)
            self.vision_proxies.xyz_points.stop()
