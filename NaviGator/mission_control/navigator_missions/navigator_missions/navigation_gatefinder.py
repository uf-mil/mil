from __future__ import annotations

import math
from dataclasses import dataclass
from enum import Enum, auto
from typing import ClassVar

import axros
from geometry_msgs.msg import Pose
from mil_msgs.msg import PerceptionObject
from std_srvs.srv import SetBoolRequest
from tf.transformations import quaternion_from_euler

from .navigator import NaviGatorMission


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

    @classmethod
    def from_perception_object(cls, perception_object: PerceptionObject):
        return cls(
            color=BuoyColor.from_label(perception_object.labeled_classification),
            pose=perception_object.pose,
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
    def mid_pose(self) -> Pose:
        mid_x = (self.left_buoy.pose.position.x + self.right_buoy.pose.position.y) / 2
        mid_y = (self.left_buoy.pose.position.y + self.right_buoy.pose.position.y) / 2
        slope = (self.left_buoy.pose.position.x - self.right_buoy.pose.position.x) / (
            self.left_buoy.pose.position.y - self.right_buoy.pose.position.y
        )
        angle = math.atan(slope)
        pose = Pose()
        pose.position.x = mid_x
        pose.position.y = mid_y
        quat = quaternion_from_euler(0, 0, angle)
        pose.orientation.x = quat[0]
        pose.orientation.y = quat[1]
        pose.orientation.z = quat[2]
        pose.orientation.w = quat[3]
        return pose

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
    required_right_color = BuoyColor.WHITE


@dataclass
class EndGate(Gate):
    required_left_color = BuoyColor.RED
    required_right_color = BuoyColor.WHITE


class GateClassifier:
    def __init__(self, mission: NavigationGatefinder):
        self.mission = mission

    def _nearby_buoys(self, buoy: Buoy) -> list[Buoy]:
        """
        Returns a list of nearby buoys, sorted by distance.
        """
        nearby = sorted(self.ordered_buoys, key=lambda b: b.distance(buoy.pose))
        return [n for n in nearby if n.distance(buoy.pose) < 20]

    def mark_completed(self, gate: Gate) -> None:
        pass

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
        while True:
            gates = await classifier.available_gates()
            self.send_feedback(f"Found {len(gates)} gates...")
            for gate in gates:
                self.send_feedback(f"Traversing {gate}...")
                await self.move.set_position(gate.mid_pose).look_at(gate.mid_pose).go()
