import mil_ros_tools
import numpy as np
import rospy
from geometry_msgs.msg import Point
from image_geometry import PinholeCameraModel
from mil_misc_tools import text_effects
from sensor_msgs.msg import CameraInfo
from std_srvs.srv import SetBool, Trigger

from .sub_singleton import SubjuGatorMission

fprint = text_effects.FprintFactory(title="BALL_DROP", msg_color="cyan").fprint

SPEED = 0.25
FAST_SPEED = 1

SEARCH_HEIGHT = 1.5
HEIGHT_BALL_DROPER = 0.75
TRAVEL_DEPTH = 0.75  # 2


class BallDrop(SubjuGatorMission):
    async def run(self, args):
        try:
            ree = rospy.ServiceProxy("/vision/garlic/enable", SetBool)
            resp = ree(True)
            if not resp:
                print("Error, failed to init neural net.")
                return
        except rospy.ServiceException:
            print("Service Call Failed")

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
        #   yield enable_service(SetBoolRequest(data=True))

        try:
            save_pois = rospy.ServiceProxy("/poi_server/save_to_param", Trigger)
            save_pois()
            if not rospy.has_param("/poi_server/initial_pois/ball_drop"):
                fprint("Forgot to add ball_drop to guess?", msg_color="yellow")
            else:
                fprint("Found ball_drop.", msg_color="green")
                await self.go(
                    self.move()
                    .set_position(
                        np.array(rospy.get_param("/poi_server/initial_pois/ball_drop")),
                    )
                    .depth(TRAVEL_DEPTH),
                    speed=FAST_SPEED,
                )
        except Exception as e:
            fprint(str(e) + "Forgot to run guess server?", msg_color="yellow")

        ball_drop_sub = self.nh.subscribe("/bbox_pub", Point)
        await ball_drop_sub.setup()
        await self.go(
            self.move().to_height(SEARCH_HEIGHT).zero_roll_and_pitch(),
            speed=SPEED,
        )

        while True:
            fprint("Getting location of ball drop...")
            ball_drop_msg = yield ball_drop_sub.get_next_message()
            ball_drop_xy = mil_ros_tools.rosmsg_to_numpy(ball_drop_msg)[:2]
            vec = ball_drop_xy - cam_center
            fprint(f"Vec: {vec}")
            vec = vec / cam_norm
            vec[1] = -vec[1]
            fprint(f"Rel move vec {vec}")
            if np.allclose(vec, np.asarray(0), atol=50):
                break
            vec = np.append(vec, 0)

            await self.move.relative_depth(vec).go(speed=SPEED)

        fprint(f"Centered, going to depth {HEIGHT_BALL_DROPER}")
        await self.move.to_height(HEIGHT_BALL_DROPER).zero_roll_and_pitch().go(
            speed=SPEED,
        )
        fprint("Dropping marker")
        await self.actuators.drop_marker()
