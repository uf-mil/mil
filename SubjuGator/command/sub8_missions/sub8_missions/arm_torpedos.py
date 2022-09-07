from __future__ import annotations

import asyncio
from typing import Optional

import numpy as np
import visualization_msgs.msg as visualization_msgs
from geometry_msgs.msg import Point, Vector3
from mil_misc_tools import FprintFactory
from mil_ros_tools import rosmsg_to_numpy
from txros import util
from visualization_msgs.msg import Marker, MarkerArray

from .sub_singleton import SubjuGator

MISSION = "Torpedo Challenge"


class Target:

    position: np.ndarray | None
    destroyed: bool

    def __init__(self):
        self.position = None
        self.destroyed = False

    def set_destroyed(self):
        self.destroyed = True

    def update_position(self, pos: np.ndarray):
        self.position = pos


class FireTorpedos(SubjuGator):
    """
    Mission to solve the torpedo RoboSub challenge.

    This code was based off of the Buoy mission code written by Kevin Allen.
    Its goal is to search for a target on the torpedo board and fire at it.
    """

    TIMEOUT_SECONDS = 15
    Z_PATTERN_RADIUS = 1
    X_PATTERN_RADIUS = 1.0
    X_OFFSET = 0.2
    Z_OFFSET = 0.2
    BACKUP_METERS = 3.0
    BLIND = True

    targets: dict[str, Target]  # Each of the targets being fired at
    done: bool  # Whether the mission has completed

    def __init__(self):
        self.print_info = FprintFactory(title=MISSION).fprint
        self.print_bad = FprintFactory(title=MISSION, msg_color="red").fprint
        self.print_good = FprintFactory(title=MISSION, msg_color="green").fprint

        # B = bottom; T = Top; L = left; R = right; C = center; O = unblocked;
        # X = blocked;
        self.targets = {
            "TCX": Target(),
            "TRX": Target(),
            "TLX": Target(),
            "BCO": Target(),
        }
        self.pattern_done = False
        self.done = False
        self.ltime = None

    def generate_pattern(self):
        z = self.Z_PATTERN_RADIUS
        X = self.X_PATTERN_RADIUS
        self.moves = [[0, X, -z], [0, -X, 0], [0, X, z], [0, -X, 0]]
        self.move_index = 0

    async def search(self):
        global markers
        markers = MarkerArray()
        pub_markers = self.nh.advertise("/torpedo/rays", MarkerArray)
        await pub_markers.setup()
        while True:
            info = "CURRENT TARGETS: "

            target = "BCO"
            pub_markers.publish(markers)
            """
            In the event we want to attempt other targets beyond bare minimum
            for target in self.targets. Eventually we will want to take the
            best target, which is the TCX target. Priority targeting will
            come once we confirm we can actually unblock currently blocked
            targets.
            """
            print("REQUESTING POSE")
            res = self.vision_proxies.xyz_points.get_pose(target="board")
            if res.found:
                print("POSE FOUND!")
                self.ltime = res.pose.header.stamp
                self.targets[target].update_position(
                    rosmsg_to_numpy(res.pose.pose.position)
                )
                self.normal = rosmsg_to_numpy(res.pose.pose.orientation)[:3]
                marker = Marker(
                    ns="torp_board",
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
            if self.targets[target].position is not None:
                print("TARGET IS NOT NONE")
                info += target + " "
            await self.nh.sleep(0.1)  # Throttle service calls
            self.print_info(info)

    async def pattern(self):
        self.print_info("Descending to Depth...")
        # await self.move.depth(1.5).go(blind=self.BLIND, speed=0.1)
        await self.move.left(2).go(blind=self.BLIND, speed=0.5)
        await self.nh.sleep(2)
        await self.move.right(2).go(blind=self.BLIND, speed=0.5)
        await self.nh.sleep(2)
        await self.move.left(1).go(blind=self.BLIND, speed=0.5)
        await self.nh.sleep(2)
        await self.move.down(0.5).go(blind=self.BLIND, speed=0.5)
        await self.nh.sleep(2)
        # def err():
        # self.print_info('Search pattern canceled')

        # self.pattern_done = False
        # for i, move in enumerate(self.moves[self.move_index:]):
        # move = self.move.relative(np.array(move)).go(blind=self.BLIND, speed=0.1)
        # await self.nh.sleep(2)
        # move.addErrback(err)
        # await move
        # self.move_index = i + 1
        # self.print_bad('Pattern finished. Firing at any locked targets.')
        # self.pattern_done = True

    async def fire(self, target: str):
        self.print_info(f"FIRING {target}")
        target_pose = self.targets[target].position
        await self.move.go(blind=self.BLIND, speed=0.1)  # Station hold
        transform = await self._tf_listener.get_transform("map", "/base_link")
        # target_position = transform._q_mat.dot(
        #         target_pose - transform._p)

        sub_pos = await self.tx_pose()
        print("Current Sub Position: ", sub_pos)

        # sub_pos = transform._q_mat.dot(
        #        (sub_pos[0]) - transform._p)
        # target_position = sub_pos[0] - target_pose
        print("Moving to Target Position: ", target_pose)
        target_position = target_pose + self.normal
        # await self.move.look_at_without_pitching(target_position).go(
        #     blind=self.BLIND, speed=.25)
        print("Target normal: ", self.normal)
        print("Point: ", target_position)
        await self.move.set_position(
            np.array([target_position[0], target_position[1], target_position[2]])
        ).go(blind=True, speed=0.5)

        self.print_good(
            "{} locked. Firing torpedoes. Hit confirmed, good job Commander.".format(
                target
            )
        )
        sub_pos = await self.tx_pose()
        print("Current Sub Position: ", sub_pos)
        await self.actuators.shoot_torpedo1()
        await self.actuators.shoot_torpedo2()
        self.done = True

    def get_target(self) -> str | None:
        """
        Returns the target we are going to focus on. Loop through priorities.
        Highest priority is the TCX target, followed by the TRX and TLX.
        Lowest priority is the BCO target. Targets are ordered in the list
        already, so this will take the first target it can actually find.
        Currently this will always by the BCO target. This is because we don't
        differentiate between targets currently as we cannot unblock the
        blocked ones. This means there is only one target for us to lock onto.
        """
        target = "BCO"
        if self.targets[target].destroyed:
            pass
            # temp = target
        elif self.targets[target].position is not None:
            return target
        elif self.pattern_done and self.targets[target].position is not None:
            return target
        else:
            return None

    async def run(self, args):
        # start_time = await self.nh.get_time()  # Store time mission
        # starts for timeout

        self.print_info("Enabling Perception")
        self.print_info(f"{self.vision_proxies.xyz_points}, Ree")
        self.vision_proxies.xyz_points.start()
        self.generate_pattern()
        pattern = asyncio.create_task(self.pattern())
        self.do_search()
        while not self.done:
            t = self.get_target()
            if t is not None:
                pattern.cancel()
                await self.fire(t)
                self.targets[t].set_destroyed()
                if not self.done:
                    pattern = self.pattern()
            elif self.pattern_done:
                break
            await self.nh.sleep(0.1)
        search.cancel()
        pattern.cancel()
        self.vision_proxies.xyz_points.stop()
        self.print_good("Done!")
