#! /usr/bin/env python3
import random

import mil_ros_tools
import numpy as np
import rospy
import visualization_msgs.msg as visualization_msgs
from geometry_msgs.msg import Point, Vector3
from mil_misc_tools import text_effects

# from sub8_msgs.srv import GuessRequest, GuessRequestRequest
from std_srvs.srv import Trigger
from visualization_msgs.msg import Marker, MarkerArray

from .sub_singleton import SubjuGator

fprint = text_effects.FprintFactory(title="PINGER", msg_color="cyan").fprint

SPEED = 0.75
FREQUENCY = 37000
FREQUENCY_TOL = 3000

PINGER_HEIGHT = 0.75  # how high to go above pinger after found
MOVE_AT_DEPTH = 0.5  # how low to swim and move

POSITION_TOL = 0.09  # how close to pinger before quitting
Z_POSITION_TOL = -0.53


class Pinger(SubjuGator):
    async def run(self, args):
        global markers
        markers = MarkerArray()
        pub_markers = self.nh.advertise("/pinger/rays", MarkerArray)
        await pub_markers.setup()

        fprint("Getting Guess Locations")

        use_prediction = False

        try:
            save_pois = rospy.ServiceProxy("/poi_server/save_to_param", Trigger)
            _ = save_pois()
            if rospy.has_param(
                "/poi_server/initial_pois/pinger_shooter"
            ) and rospy.has_param("/poi_server/initial_pois/pinger_surface"):
                fprint("Found two pinger guesses", msg_color="green")
                pinger_1_req = rospy.get_param(
                    "/poi_server/initial_pois/pinger_surface"
                )
                pinger_2_req = rospy.get_param(
                    "/poi_server/initial_pois/pinger_shooter"
                )

                # check \/
                pinger_guess = await self.transform_to_baselink(
                    self, pinger_1_req, pinger_2_req
                )
                fprint(pinger_guess)
            else:
                use_prediction = False
                fprint("Forgot to add pinger to guess server?", msg_color="yellow")
        except Exception as e:
            fprint(
                "Failed to /guess_location. Procceding without guess",
                msg_color="yellow",
            )
            use_prediction = False
            fprint(e)

        while True:
            fprint("=" * 50)

            fprint("waiting for message")
            p_message = await self.pinger_sub.get_next_message()

            pub_markers.publish(markers)
            fprint(p_message)

            # Ignore freuqnecy
            if not abs(p_message.freq - FREQUENCY) < FREQUENCY_TOL:
                fprint(
                    f"Ignored! Received Frequency {p_message.freq}",
                    msg_color="red",
                )
                if use_prediction:
                    fprint("Moving to random guess")
                    await self.go_to_random_guess(self, pinger_1_req, pinger_2_req)
                continue

            # Ignore magnitude from processed ping
            p_position = mil_ros_tools.rosmsg_to_numpy(p_message.position)
            vec = p_position / np.linalg.norm(p_position)
            if np.isnan(vec).any():
                fprint("Ignored! nan", msg_color="red")
                if use_prediction:
                    await self.go_to_random_guess(self, pinger_1_req, pinger_2_req)
                continue

            # Transform hydrophones vector to relative coordinate
            transform = await self._tf_listener.get_transform(
                "/base_link", "/hydrophones"
            )
            vec = transform._q_mat.dot(vec)

            fprint(f"Transformed vec: {vec}")
            marker = Marker(
                ns="pinger",
                action=visualization_msgs.Marker.ADD,
                type=Marker.ARROW,
                scale=Vector3(0.2, 0.5, 0),
                points=np.array([Point(0, 0, 0), Point(vec[0], vec[1], vec[2])]),
            )
            marker.id = 3
            marker.header.frame_id = "/base_link"
            marker.color.r = 1
            marker.color.g = 0
            marker.color.a = 1
            markers.markers.append(marker)

            # Check if we are on top of pinger
            if (
                abs(vec[0]) < POSITION_TOL
                and abs(vec[1]) < POSITION_TOL
                and vec[2] < Z_POSITION_TOL
            ):
                if not use_prediction:
                    fprint("Arrived to pinger!")
                    break

                sub_position, _ = await self.tx_pose()
                dists = [
                    np.linalg.norm(
                        sub_position
                        - mil_ros_tools.rosmsg_to_numpy(x.location.pose.position)
                    )
                    for x in (pinger_1_req, pinger_2_req)
                ]
                pinger_id = np.argmin(dists)
                # pinger_id 0 = pinger_surface
                # pinger_id 1 = pinger_shooter
                await self.nh.set_param("pinger_where", int(pinger_id))

                break

            vec[2] = 0
            if use_prediction:
                pinger_guess = await self.transform_to_baselink(
                    self, pinger_1_req, pinger_2_req
                )
                fprint(f"Transformed guess: {pinger_guess}")
                # Check if the pinger aligns with guess
                # check, vec = self.check_with_guess(vec, pinger_guess)

            fprint(f"move to {vec}")
            await self.fancy_move(self, vec)

        fprint("Arrived to hydrophones! Going down!")
        await self.move.to_height(PINGER_HEIGHT).zero_roll_and_pitch().go(speed=0.1)

    async def fancy_move(self, sub: SubjuGator, vec):
        global markers
        marker = Marker(
            ns="pinger",
            action=visualization_msgs.Marker.ADD,
            type=Marker.ARROW,
            scale=Vector3(0.2, 0.3, 0.1),
            points=np.array([Point(0, 0, 0), Point(vec[0], vec[1], vec[2])]),
        )
        marker.id = 4
        marker.header.frame_id = "/base_link"
        marker.color.r = 0
        marker.color.g = 0
        marker.color.b = 1
        marker.color.a = 1
        markers.markers.append(marker)

        await sub.move.relative(vec).depth(MOVE_AT_DEPTH).zero_roll_and_pitch().go(
            speed=SPEED
        )

    async def go_to_random_guess(self, sub: SubjuGator, pinger_1_req, pinger_2_req):
        pinger_guess = await self.transform_to_baselink(sub, pinger_1_req, pinger_2_req)
        where_to = random.choice(pinger_guess)
        where_to = where_to / np.linalg.norm(where_to)
        fprint(f"Going to random guess {where_to}", msg_color="yellow")
        await self.fancy_move(sub, where_to)

    def check_with_guess(self, vec: np.ndarray, pinger_guess: np.ndarray):
        for guess in pinger_guess:
            guess[2] = 0
        dots = [vec.dot(guess / np.linalg.norm(guess)) for guess in pinger_guess]
        fprint(f"Dots {dots}")
        if dots[0] < 0.6 and dots[1] < 0.6:
            # Get the guess that is close to pnger vec
            go_to_guess = pinger_guess[np.argmax(dots)]
            go_to_guess = go_to_guess / np.linalg.norm(go_to_guess)
            fprint(
                "Though ping was behind. Going to pinger guess {} at {}".format(
                    np.argmax(dots) + 1, go_to_guess
                ),
                msg_color="yellow",
            )
            return (False, go_to_guess)
        return (True, vec)

    async def transform_to_baselink(self, sub: SubjuGator, pinger_1_req, pinger_2_req):
        transform = await sub._tf_listener.get_transform("/base_link", "map")
        position = await sub.pose.position
        pinger_guess = [
            transform._q_mat.dot(np.array(x) - position)
            for x in (pinger_1_req, pinger_2_req)
        ]
        for idx, guess in enumerate(pinger_guess):
            marker = Marker(
                ns="pinger",
                action=visualization_msgs.Marker.ADD,
                type=Marker.ARROW,
                scale=Vector3(0.1, 0.2, 0),
                points=np.array([Point(0, 0, 0), Point(guess[0], guess[1], guess[2])]),
            )
            marker.id = idx
            marker.header.frame_id = "/base_link"
            marker.color.r = 0
            marker.color.g = 1
            marker.color.a = 1
            global markers
            markers.markers.append(marker)
        return pinger_guess
