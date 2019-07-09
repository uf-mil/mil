#!/usr/bin/env python
from navigator import Navigator
import txros
from twisted.internet import defer
from geometry_msgs.msg import PoseStamped
from mil_tools import pose_to_numpy
import numpy as np


class TrackTarget(Navigator):
    '''
    Mission to track the detect deliver target
    '''
    # Offset from the shooter to the target X, Y, Z in boat's frame conventions
    OFFSET = np.array([0., 6.5, 0.], dtype=float)
    # Number of shots to fire
    NUMBER_SHOTS = 1
    # Distance from goal pose to fire at
    DISTANCE_TOLERANCE = 0.1

    @classmethod
    @txros.util.cancellableInlineCallbacks
    def init(cls):
        # Store pose of shooter for later
        cls.base_link_to_shooter = -(yield cls.tf_listener.get_transform("base_link", "shooter"))._p
        cls.base_link_to_shooter[2] = 0.
        # Subscribe to pose
        cls.target_pose_sub = cls.nh.subscribe("/detect_deliver_target_detector/pose", PoseStamped)

    @txros.util.cancellableInlineCallbacks
    def run(self, parameters):
        goal = None
        reload_wait = None
        fired = 0

        # Continuously align until all shots are fired
        while fired < self.NUMBER_SHOTS:
            if reload_wait is not None:
                select = defer.DeferredList([self.target_pose_sub.get_next_message(), self.tx_pose, reload_wait],
                                            fireOnOneCallback=True, fireOnOneErrback=True)
            else:
                select = defer.DeferredList([self.target_pose_sub.get_next_message(), self.tx_pose],
                                            fireOnOneCallback=True, fireOnOneErrback=True)
            result, index = yield select

            # New target pose
            if index == 0:
                # Get pose of target from computer vision
                pose = result
                # Convert to numpy
                pos, quat = pose_to_numpy(pose.pose)
                # Ensures some math bag doesn't occur, not sure why. Hacks!
                quat = np.abs(quat)

                # Transform pose to ENU
                transform = yield self.tf_listener.get_transform("enu", pose.header.frame_id, pose.header.stamp)
                pos, quat = transform.transform_point(pos), transform.transform_quaternion(quat)
                pos[2] = 0.

                # Assemble Move:
                # Start directly on target but rotated so shooter faces target
                move = self.move.set_position(pos).set_orientation(quat).yaw_left(3.14)
                # Adjust for position of shooter
                move = move.rel_position(self.base_link_to_shooter)
                # Offset to optimize trajectory of launcher
                move = move.rel_position(self.OFFSET)
                # Force z=0
                move.position[2] = 0.
                # Set new goal
                goal = move.position
                # Command move to this goal
                yield move.go(move_type="bypass")

            # New odometry, see if we are close enough to goal to fire
            elif index == 1:
                # Ignore if no goal set
                if goal is None:
                    continue
                # Get distance from boat to goal
                result[0][2] = 0.
                distance = np.linalg.norm(goal - result[0])
                yield self.send_feedback("{} from goal".format(distance))

                # If close enough, fire
                if distance < self.DISTANCE_TOLERANCE:
                    # If still reloading, wait
                    if reload_wait is not None:
                        yield self.send_feedback("Aligned and waiting for reload")
                        continue
                    # Otherwise fire
                    yield self.send_feedback("Aligned. Firing!")
                    yield self.fire_launcher()
                    fired += 1
                    reload_wait = self.reload_launcher()

            # Reload finished
            elif index == 2:
                yield self.send_feedback("Reloaded.")
                reload_wait = None

        # Ensure mission exits after having reloaded
        if reload_wait is not None:
            self.send_feedback("Waiting for final reload.")
            yield reload_wait
        defer.returnValue("All balls fired.")
