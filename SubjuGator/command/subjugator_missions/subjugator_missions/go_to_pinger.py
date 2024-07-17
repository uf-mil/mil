import math

from mil_msgs.msg import DepthStamped
from mil_passive_sonar.msg import ProcessedPing
from nav_msgs.msg import Odometry
from vision_stack.msg import ObjectDetections

from .sub_singleton import SubjuGatorMission

IN_GAZEBO = True
FRAME_ID = "hydrophones"

IDEAL_DEPTH = 1  # meters
ANGLE_TO_FACE_PING = 270  # degrees

SPEED_LIMIT_VERT = 0.2
SPEED_LIMIT_YAW = 0.5


class FollowPinger(SubjuGatorMission):
    async def run(self, args):
        # Set up messages
        self.detections_sub = self.nh.subscribe(
            "/yolo_detections/1/objectDetection_last_2/analysis",
            ObjectDetections,
        )
        await self.detections_sub.setup()

        self.pinger_sub = self.nh.subscribe("/hydrophones/processed", ProcessedPing)
        await self.pinger_sub.setup()

        self.depth_sub = self.nh.subscribe("/depth", DepthStamped)
        await self.depth_sub.setup()

        self.odom_sub = self.nh.subscribe("/odom", Odometry)
        await self.odom_sub.setup()

        # Get to an ideal depth
        await self.achieve_ideal_depth()

        # Follow a pinger until certain condition is met
        await self.follow_ping()

        print("Done!")

    async def achieve_ideal_depth(self):
        depth = await self.depth_sub.get_next_message()
        depth = depth.depth

        delta = depth - IDEAL_DEPTH

        await self.go(
            self.move().up(delta).zero_roll_and_pitch(),
            speed=SPEED_LIMIT_VERT,
        )

    async def follow_ping(self):

        while True:

            ping = await self.pinger_sub.get_next_message()
            ping_vector = None

            # Extract pinger data
            if IN_GAZEBO and ping.header.frame_id == FRAME_ID or not IN_GAZEBO:
                ping_vector = ping.position
            elif IN_GAZEBO:
                continue

            odom_quat = await self.odom_sub.get_next_message()
            odom_quat = odom_quat.pose.pose.orientation

            # Orient self to the pinger in the x y
            angle_of_ping = self.cartesian_to_degrees(ping_vector.x, ping_vector.y)
            angle_of_sub = self.quaternion_to_degrees(
                odom_quat.x,
                odom_quat.y,
                odom_quat.z,
                odom_quat.w,
            )

            ang_diff = angle_of_ping - ANGLE_TO_FACE_PING

            print("Angle of pinger: ", angle_of_ping, "Angle of sub: ", angle_of_sub)
            print("Ping_vector: ", ping_vector)

            if abs(ang_diff) > 1:
                print("here")
                await self.go(
                    self.move().yaw_left_deg(ang_diff).zero_roll_and_pitch(),
                    speed=SPEED_LIMIT_YAW,
                )
            else:
                # Move forward 0.1 and check detections
                await self.go(
                    self.move().forward(0.1).zero_roll_and_pitch(),
                    speed=1,
                )

            # Check if we are at task

    def cartesian_to_degrees(self, x, y):
        # Calculate the angle in radians
        theta_radians = math.atan2(y, x)

        # Convert radians to degrees
        theta_degrees = math.degrees(theta_radians)

        # Ensure the angle is within [0, 360) degrees
        if theta_degrees < 0:
            theta_degrees += 360

        return theta_degrees

    def quaternion_to_degrees(self, x, y, z, w):
        # Normalize quaternion
        norm = math.sqrt(w * w + x * x + y * y + z * z)
        w /= norm

        # Calculate angle in radians
        theta_radians = 2 * math.acos(w)

        # Convert radians to degrees
        theta_degrees = math.degrees(theta_radians)

        return theta_degrees

    def quaternion_to_rotation_axis(self, x, y, z, w):
        # Normalize quaternion
        norm = math.sqrt(x * x + y * y + z * z + w * w)
        x /= norm
        y /= norm
        z /= norm

        return x, y, z
