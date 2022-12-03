import mil_ros_tools
import numpy as np
import rospy
from geometry_msgs.msg import Point
from image_geometry import PinholeCameraModel
from mil_misc_tools import text_effects
from sensor_msgs.msg import CameraInfo
from std_srvs.srv import Trigger
from sub8_msgs.srv import GuessRequestRequest

from .sub_singleton import SubjuGatorMission

fprint = text_effects.FprintFactory(title="DRACULA_GRAB", msg_color="cyan").fprint

SPEED = 0.25
FAST_SPEED = 1

SEARCH_HEIGHT = 1.5
HEIGHT_DRACULA_GRABBER = 0.5
TRAVEL_DEPTH = 0.5  # 2


class DraculaGrabber(SubjuGatorMission):
    async def run(self, args):
        fprint("Enabling cam_ray publisher")

        await self.nh.sleep(1)

        fprint("Connecting camera")

        cam_info_sub = self.nh.subscribe("/camera/down/camera_info", CameraInfo)
        await cam_info_sub.setup()

        fprint("Obtaining cam info message")
        cam_info = await cam_info_sub.get_next_message()
        cam_center = np.array([cam_info.width / 2, cam_info.height / 2])
        cam_norm = np.sqrt(cam_center[0] ** 2 + cam_center[1] ** 2)
        fprint(f"Cam center: {cam_center}")

        model = PinholeCameraModel()
        model.fromCameraInfo(cam_info)

        #   enable_service = self.nh.get_service_client("/vamp/enable", SetBool)
        #   await enable_service(SetBoolRequest(data=True))

        try:
            save_pois = rospy.ServiceProxy("/poi_server/save_to_param", Trigger)
            save_pois()
            if not rospy.has_param("/poi_server/initial_pois/dracula"):
                dracula_req = await vamp_axros(GuessRequestRequest(item="dracula"))
                use_prediction = False
                fprint("Forgot to add dracula to guess?", msg_color="yellow")
            else:
                fprint("Found dracula.", msg_color="green")
                await self.move.set_position(
                    np.array(rospy.get_param("/poi_server/initial_pois/dracula"))
                ).depth(TRAVEL_DEPTH).go(speed=FAST_SPEED)
        except Exception as e:
            fprint(str(e) + "Forgot to run guess server?", msg_color="yellow")

        dracula_sub = self.nh.subscribe("/bbox_pub", Point)
        await dracula_sub.setup()
        await self.move.to_height(SEARCH_HEIGHT).zero_roll_and_pitch().go(speed=SPEED)

        while True:
            fprint("Getting location of ball drop...")
            dracula_msg = await dracula_sub.get_next_message()
            dracula_xy = mil_ros_tools.rosmsg_to_numpy(dracula_msg)[:2]
            vec = dracula_xy - cam_center
            fprint(f"Vec: {vec}")
            vec = vec / cam_norm
            vec[1] = -vec[1]
            fprint(f"Rel move vec {vec}")
            if np.allclose(vec, np.asarray(0), atol=50):
                break
            vec = np.append(vec, 0)

            await self.move.relative_depth(vec).go(speed=SPEED)

        fprint(f"Centered, going to depth {HEIGHT_DRACULA_GRABBER}")
        await self.move.to_height(HEIGHT_DRACULA_GRABBER).zero_roll_and_pitch().go(
            speed=SPEED
        )
        fprint("Dropping marker")
        await self.actuators.gripper_close()
