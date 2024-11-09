from __future__ import annotations

import math
from dataclasses import dataclass
from enum import Enum, auto
from typing import ClassVar

import axros
import numpy as np
import rospy
from geometry_msgs.msg import Pose
from mil_msgs.msg import PerceptionObject
from navigator_msgs.srv import MessageFollowPath, MessageFollowPathRequest
from std_srvs.srv import SetBoolRequest
from tf.transformations import quaternion_from_euler

from .navigator import NaviGatorMission

FAILURE_THRESHOLD = 2000


class BuoyColor(Enum):
    RED = auto()
    GREEN = auto()
    WHITE = auto()
    UNKNOWN = auto()

    @classmethod
    def from_label(cls, label: str):
        if "red" in label.lower():
            return cls.RED
        if "green" in label.lower():
            return cls.GREEN
        if "white" in label.lower():
            return cls.WHITE
        return cls.UNKNOWN


@dataclass
class Buoy:
    color: BuoyColor
    pose: Pose
    id: int

    @classmethod
    def from_perception_object(cls, perception_object: PerceptionObject):
        return cls(
            color=BuoyColor.from_label(perception_object.labeled_classification),
            pose=perception_object.pose,
            id=perception_object.id,
        )

    def xyz(self):
        return np.array(
            [self.pose.position.x, self.pose.position.y, self.pose.position.z],
        )

    def distance(self, pose: Pose) -> float:
        return math.sqrt(
            ((pose.position.x - self.pose.position.x) ** 2)
            + ((pose.position.y - self.pose.position.y) ** 2),
        )


@dataclass
class Gate:
    left_buoy: Buoy
    right_buoy: Buoy
    traversed: bool = False
    required_left_color: ClassVar[BuoyColor]
    required_right_color: ClassVar[BuoyColor]

    @property
    def mid_pose(self):
        # Calculate the midpoint of the found buoys
        mid_x = (self.left_buoy.pose.position.x + self.right_buoy.pose.position.x) / 2
        mid_y = (self.left_buoy.pose.position.y + self.right_buoy.pose.position.y) / 2
        slope = (self.left_buoy.pose.position.x - self.right_buoy.pose.position.x) / (
            self.left_buoy.pose.position.y - self.right_buoy.pose.position.y
        )
        angle = math.atan(slope) + math.pi / 2
        pose = Pose()
        pose.position.x = mid_x
        pose.position.y = mid_y
        quat = quaternion_from_euler(0, 0, angle)
        pose.orientation.x = quat[0]
        pose.orientation.y = quat[1]
        pose.orientation.z = quat[2]
        pose.orientation.w = quat[3]
        return (mid_x, mid_y, 0), quat

    @classmethod
    def other_color(cls, color_one: BuoyColor) -> BuoyColor:
        if color_one == cls.required_left_color:
            return cls.required_right_color
        if color_one == cls.required_right_color:
            return cls.required_left_color
        raise RuntimeError(f"Color {color_one} cannot form this gate.")


@dataclass
class MiddleGate(Gate):
    required_left_color = BuoyColor.RED
    required_right_color = BuoyColor.GREEN


@dataclass
class StartGate(Gate):
    required_left_color = BuoyColor.RED
    required_right_color = BuoyColor.GREEN


@dataclass
class EndGate(Gate):
    required_left_color = BuoyColor.RED
    required_right_color = BuoyColor.GREEN


class GateClassifier:
    def __init__(self, mission: NavigationGatefinder):
        self.mission = mission
        self.trajectory = []
        self.ids_invesitigated = set()
        self.failures_to_move = 0
        self.midpoint_lengths = []

    def _nearby_buoys(self, buoy: Buoy) -> list[Buoy]:
        """
        Returns a list of nearby buoys, sorted by distance.
        """
        nearby = sorted(self.ordered_buoys, key=lambda b: b.distance(buoy.pose))
        return [n for n in nearby if n.distance(buoy.pose) < 20]

    def mark_completed(self, gate: Gate) -> None:
        pass

    async def get_next_gate(self, boat_pose) -> Gate | int:
        # Get all database objects
        objects: list[PerceptionObject] = (
            await self.mission.database_query(name="all")
        ).objects

        # get nearest red and nearest green buoys
        nearest_red: None | Buoy = None
        nearest_green: None | Buoy = None
        for db_object in objects:
            # Could we do this more efficiently with a different database request?
            DB_BUOY = Buoy.from_perception_object(db_object)
            if (
                db_object.labeled_classification == "red_cylinder"
                and DB_BUOY.id not in self.ids_invesitigated
            ):
                if nearest_red:
                    distance = np.linalg.norm(DB_BUOY.xyz() - boat_pose)
                    nearest_red_distance = np.linalg.norm(nearest_red.xyz() - boat_pose)
                    nearest_red = (
                        DB_BUOY if distance < nearest_red_distance else nearest_red
                    )
                else:
                    nearest_red = DB_BUOY

            if (
                db_object.labeled_classification == "green_cylinder"
                and DB_BUOY.id not in self.ids_invesitigated
            ):
                if nearest_green:
                    distance = np.linalg.norm(DB_BUOY.xyz() - boat_pose)
                    nearest_green_distance = np.linalg.norm(
                        nearest_green.xyz() - boat_pose,
                    )
                    nearest_green = (
                        DB_BUOY if distance < nearest_green_distance else nearest_green
                    )
                else:
                    nearest_green = DB_BUOY

        # calculate midpoint between both points
        # if ONLY green then go to a point 5 meters from the right of the green buoy
        # if ONLY red then go to a point 5 meters from the left of the red buoy

        # mark those buoys as investigated
        mid_point = None
        if nearest_green and nearest_red:
            self.ids_invesitigated.add(nearest_red.id)
            self.ids_invesitigated.add(nearest_green.id)
            mid_x = (nearest_green.pose.position.x + nearest_red.pose.position.x) / 2
            mid_y = (nearest_green.pose.position.y + nearest_red.pose.position.y) / 2
            mid_point = (mid_x, mid_y, 0)
            self.trajectory.append(mid_point)

        # missing red
        elif nearest_green:
            rospy.logerr(
                f"Could not find red for green({nearest_green.id}), here is the boats pos:\n{boat_pose}",
            )
            # Get nearest red to the green
            objects: list[PerceptionObject] = (
                await self.mission.database_query(name="all")
            ).objects
            nearest_red = None
            for db_object in objects:
                # Could we do this more efficiently with a different database request?
                rospy.logerr(db_object.labeled_classification)
                DB_BUOY = Buoy.from_perception_object(db_object)
                if (
                    db_object.labeled_classification == "red_cylinder"
                    and DB_BUOY.id not in self.ids_invesitigated
                ):
                    if nearest_red:
                        distance = np.linalg.norm(DB_BUOY.xyz() - nearest_green.xyz())
                        nearest_red_distance = np.linalg.norm(
                            nearest_red.xyz() - nearest_green.xyz(),
                        )
                        nearest_red = (
                            DB_BUOY if distance < nearest_red_distance else nearest_red
                        )
                    else:
                        nearest_red = DB_BUOY
            # Set midpoint if nearest red was found
            if nearest_red:
                mid_x = (
                    nearest_green.pose.position.x + nearest_red.pose.position.x
                ) / 2
                mid_y = (
                    nearest_green.pose.position.y + nearest_red.pose.position.y
                ) / 2
                mid_point = (mid_x, mid_y, 0)
                self.trajectory.append(mid_point)

        # missing green
        elif nearest_red:
            rospy.logerr(
                f"Could not find green for red({nearest_red.id}), here is the boats pos:\n{boat_pose}",
            )
            # Get nearest green to the red
            objects: list[PerceptionObject] = (
                await self.mission.database_query(name="all")
            ).objects
            nearest_green = None
            for db_object in objects:
                DB_BUOY = Buoy.from_perception_object(db_object)
                if (
                    db_object.labeled_classification == "green_cylinder"
                    and DB_BUOY.id not in self.ids_invesitigated
                ):
                    if nearest_green:
                        distance = np.linalg.norm(DB_BUOY.xyz() - nearest_red.xyz())
                        nearest_green_distance = np.linalg.norm(
                            nearest_green.xyz() - nearest_red.xyz(),
                        )
                        nearest_green = (
                            DB_BUOY
                            if distance < nearest_green_distance
                            else nearest_green
                        )
                    else:
                        nearest_green = DB_BUOY
            # Set midpoint if nearest red was found
            if nearest_green:
                mid_x = (
                    nearest_green.pose.position.x + nearest_red.pose.position.x
                ) / 2
                mid_y = (
                    nearest_green.pose.position.y + nearest_red.pose.position.y
                ) / 2
                mid_point = (mid_x, mid_y, 0)
                self.trajectory.append(mid_point)

        rospy.logerr(mid_point)

        # return next move point
        if mid_point and nearest_green and nearest_red:
            self.failures_to_move = 0
            rospy.logerr(f"nearest red: {nearest_red.id}")
            rospy.logerr(f"nearest green: {nearest_green.id}")
            self.trajectory.append(mid_point)
            self.midpoint_lengths.append(
                math.sqrt(
                    (nearest_green.pose.position.x - mid_point[0]) ** 2
                    + (nearest_green.pose.position.y - mid_point[1]) ** 2,
                ),
            )
            return Gate(nearest_red, nearest_green)

        if self.failures_to_move > FAILURE_THRESHOLD:
            rospy.logerr("FAILURE THRESHOLD REACHED")
            return -1
        #     from sklearn.preprocessing import PolynomialFeatures
        #     from sklearn.linear_model import LinearRegression
        #     import matplotlib.pyplot as plt

        #     points=np.array(self.trajectory)
        #     x = points[:, 0].reshape(-1, 1)  # Reshape for sklearn
        #     y = points[:, 1]
        #     # Follow the trajectory until you reach the farthest cylinder that is within the
        #     poly = PolynomialFeatures(degree=2)  # You can change the degree as needed
        #     x_poly = poly.fit_transform(x)

        #     # Fit the polynomial regression model
        #     model = LinearRegression()
        #     model.fit(x_poly, y)
        #     x_pred = np.linspace(min(x), max(x), 100)

        #     # Predict using the transformed features
        #     y_pred = model.predict(poly.transform(x_pred))

        #     # Plot the results
        #     plt.scatter(x, y, color='blue', label='Original Points')
        #     plt.plot(x_pred, y_pred, color='red', label='Predicted Trajectory')
        #     plt.xlabel('X Position')
        #     plt.ylabel('Y Position')
        #     plt.title('Robot Trajectory Prediction (Polynomial)')
        #     plt.legend()
        #     plt.show()

        self.failures_to_move += 1
        return 0

    async def available_gates(self) -> list[Gate]:
        """
        Returns a list of the gates that still need to be traversed.
        """
        gates: list[Gate] = []
        objects: list[PerceptionObject] = (
            await self.mission.database_query(name="all")
        ).objects
        found_buoys: set[Buoy] = set()
        for db_object in objects:
            # Could we do this more efficiently with a different database request?
            if "buoy" not in db_object.labeled_classification:
                continue
            # Ignore circular buoys
            if "circular" in db_object.labeled_classification:
                continue
            found_buoys.add(Buoy.from_perception_object(db_object))

        # Order the buoys by distance (to form gates easier)
        pose = self.mission.pose
        assert pose is not None
        self.ordered_buoys: list[Buoy] = sorted(
            found_buoys,
            key=lambda b: b.distance(pose),
        )

        # Turn the buoys into gates
        handled_buoys: set[Buoy] = set()
        print(f"found {len(found_buoys)}...")
        for buoy in found_buoys:
            # Ignore if already a part of another gate
            if buoy in handled_buoys:
                continue
            # Find all nearby buoys that could potentially form a gates
            nearby_buoys = self._nearby_buoys(buoy)
            # Which gate should we be making?
            intended_gate = MiddleGate
            if len(gates) == 0:
                intended_gate = StartGate
            other_color = intended_gate.other_color(buoy.color)
            nearby_colored_buoys = [b for b in nearby_buoys if b.color == other_color]
            if len(nearby_colored_buoys) == 0:
                raise RuntimeError(
                    "Can't find any nearby buoys matching the color needed for this gate!",
                )
            closest_nearest_buoy = nearby_colored_buoys[0]
            gates.append(intended_gate(buoy, closest_nearest_buoy))
            handled_buoys.add(buoy)
            handled_buoys.add(closest_nearest_buoy)
        return gates


class NavigationGatefinder(NaviGatorMission):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)

    async def run(self, parameters):
        await self.change_wrench("autonomous")
        await axros.util.wrap_time_notice(
            self.set_classifier_enabled.wait_for_service(),
            4,
            "classifier enable service",
        )
        await self.set_classifier_enabled(SetBoolRequest(True))
        # start_pose = await self.tx_pose()
        classifier = GateClassifier(self)
        td_feedback = self.nh.get_service_client(
            "/follow_path_message",
            MessageFollowPath,
        )

        while True:
            rospy.logerr(classifier.ids_invesitigated)
            next_gate = await classifier.get_next_gate(self.pose[0])
            try:
                await axros.wrap_timeout(td_feedback.wait_for_service(), duration=5)
                await td_feedback(MessageFollowPathRequest(entry_color="R", finished=1))
            except TimeoutError:
                self.send_feedback("no feedback")
            self.send_feedback("got next gate!")
            if isinstance(next_gate, Gate):
                await self.move.set_position(next_gate.mid_pose[0]).set_orientation(
                    next_gate.mid_pose[1],
                ).go()
            elif next_gate == -1:
                rospy.loginfo("All Done!")
                break

        try:
            await axros.wrap_timeout(td_feedback.wait_for_service(), duration=5)
            await td_feedback(MessageFollowPathRequest(entry_color="R", finished=2))
        except TimeoutError:
            self.send_feedback("no feedback")
