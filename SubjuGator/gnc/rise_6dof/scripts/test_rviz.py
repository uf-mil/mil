#!/usr/bin/env python3
import math

import numpy
import roslib
import rospy
from geometry_msgs.msg import (
    Point,
    Pose,
    PoseWithCovariance,
    Quaternion,
    Twist,
    TwistWithCovariance,
    Vector3,
    WrenchStamped,
)
from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from mil_msgs.msg import PoseTwist, PoseTwistStamped
from mil_msgs.orientation_helpers import xyz_array, xyzw_array
from nav_msgs.msg import Odometry
from std_msgs.msg import ColorRGBA, Header
from tf import transformations
from visualization_msgs.msg import (
    InteractiveMarker,
    InteractiveMarkerControl,
    InteractiveMarkerFeedback,
    Marker,
)


def makeBoxControl() -> InteractiveMarkerControl:
    return InteractiveMarkerControl(
        always_visible=True,
        markers=[
            Marker(
                type=Marker.CUBE,
                scale=Vector3(0.45, 0.45, 0.45),
                color=ColorRGBA(0.5, 0.5, 0.5, 1),
            ),
        ],
    )


def make6DofMarker(
    name: str,
    description: str,
    allow_rotation: bool = False,
) -> InteractiveMarker:
    int_marker = InteractiveMarker()
    int_marker.name = name
    int_marker.description = description
    int_marker.header.frame_id = "map"
    int_marker.scale = 1

    # insert a box
    int_marker.controls.append(makeBoxControl())

    for direction in "xyz":
        control = InteractiveMarkerControl()
        control.orientation.w = 1
        control.orientation.x = direction == "x"
        control.orientation.y = direction == "y"
        control.orientation.z = direction == "z"
        control.name = "move_" + direction
        control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        int_marker.controls.append(control)

        if allow_rotation:
            control = InteractiveMarkerControl()
            control.orientation.w = 1
            control.orientation.x = direction == "x"
            control.orientation.y = direction == "y"
            control.orientation.z = direction == "z"
            control.name = "rotate_" + direction
            control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
            int_marker.controls.append(control)

    return int_marker


def makeMovingMarker(name: str, description: str) -> InteractiveMarker:
    int_marker = InteractiveMarker()
    int_marker.name = name
    int_marker.description = description
    int_marker.header.frame_id = "map"
    int_marker.scale = 1

    # insert a box
    int_marker.controls.append(makeBoxControl())

    return int_marker


if __name__ == "__main__":
    roslib.load_manifest("interactive_markers")
    roslib.load_manifest("rise_6dof")

    rospy.init_node("test_rviz")
    server = InteractiveMarkerServer("test_rviz")

    wrench = [(0, 0, 0), (0, 0, 0)]

    def set_wrench(new_wrench):
        assert new_wrench.header.frame_id == "/base_link"
        new_wrench = new_wrench.wrench
        wrench[:] = [
            (new_wrench.force.x, new_wrench.force.y, new_wrench.force.z),
            (new_wrench.torque.x, new_wrench.torque.y, new_wrench.torque.z),
        ]

    wrench_sub = rospy.Subscriber("/output", WrenchStamped, set_wrench)

    desired_pub = rospy.Publisher("/desired", PoseTwistStamped)
    desired_posetwist = PoseTwistStamped(
        header=Header(frame_id="map", stamp=rospy.Time.now()),
        posetwist=PoseTwist(
            pose=Pose(position=Point(0, 0, 0), orientation=Quaternion(0, 0, 0, 1)),
            twist=Twist(linear=Vector3(0, 0, 0), angular=Vector3(0, 0, 0)),
        ),
    )

    def updateDesired(feedback=None):
        global desired_posetwist
        if (
            feedback is not None
            and feedback.event_type == InteractiveMarkerFeedback.POSE_UPDATE
        ):
            # compute twist from this and last pose update
            if desired_posetwist is not None and rospy.Duration.from_sec(
                0,
            ) < rospy.Time.now() - desired_posetwist.header.stamp < rospy.Duration.from_sec(
                0.05,
            ):
                dt = (rospy.Time.now() - desired_posetwist.header.stamp).to_sec()
                vel = (
                    xyz_array(feedback.pose.position)
                    - xyz_array(desired_posetwist.posetwist.pose.position)
                ) / dt

                def quat_to_scaledaxis(q):
                    q = q / numpy.linalg.norm(q)
                    if q[3] < 0:
                        q *= -1
                    return 2 / numpy.sinc(q[3] / math.pi) * numpy.array(q[:3])

                angvel = (
                    quat_to_scaledaxis(
                        transformations.quaternion_multiply(
                            xyzw_array(feedback.pose.orientation),
                            transformations.quaternion_conjugate(
                                xyzw_array(
                                    desired_posetwist.posetwist.pose.orientation,
                                ),
                            ),
                        ),
                    )
                    / dt
                )
                twist = Twist(linear=Vector3(*vel), angular=Vector3(*angvel))
            else:
                twist = Twist(linear=Vector3(0, 0, 0), angular=Vector3(0, 0, 0))
            desired_posetwist = PoseTwistStamped(
                header=Header(frame_id="map", stamp=rospy.Time.now()),
                posetwist=PoseTwist(
                    pose=feedback.pose,
                    twist=twist,
                ),
            )
        if rospy.Time.now() - desired_posetwist.header.stamp > rospy.Duration.from_sec(
            0.05,
        ):
            desired_posetwist.posetwist.twist = Twist(
                linear=Vector3(0, 0, 0),
                angular=Vector3(0, 0, 0),
            )
        if desired_posetwist is not None:
            desired_pub.publish(desired_posetwist)

    server.insert(make6DofMarker("desired", "\nDesired", True), updateDesired)
    rospy.Timer(rospy.Duration(0.01), lambda msg: updateDesired(None))

    current_pub = rospy.Publisher("/current", Odometry)
    server.insert(makeMovingMarker("current", "Current\n"))

    current_pos = numpy.zeros(3)
    current_vel = numpy.zeros(3)
    current_orientation = numpy.array([0, 0, 0, 1])
    current_angvel = numpy.zeros(3)
    vel_m = 10
    angvel_I = 10
    dt = 0.01

    def updateCurrent(msg):
        global current_pos, current_vel, current_orientation, current_angvel

        world_from_body = transformations.quaternion_matrix(current_orientation)[:3, :3]

        # print wrench
        wrench_world = world_from_body.dot(wrench[0]), world_from_body.dot(wrench[1])

        current_vel += dt * (wrench_world[0] - 10 * current_vel) / vel_m
        current_angvel += dt * (wrench_world[1] - 5 * current_angvel) / angvel_I

        current_pos += dt * current_vel

        def quat_exp(quat):
            x, y, z = quat
            return transformations.quaternion_about_axis(
                numpy.linalg.norm((x, y, z)),
                (x, y, z),
            )

        current_orientation = transformations.quaternion_multiply(
            quat_exp(current_angvel * dt),
            current_orientation,
        )

        pose = Pose(
            Point(x=current_pos[0], y=current_pos[1], z=current_pos[2]),
            Quaternion(
                x=current_orientation[0],
                y=current_orientation[1],
                z=current_orientation[2],
                w=current_orientation[3],
            ),
        )

        server.setPose("current", pose)
        server.applyChanges()

        current_pub.publish(
            Odometry(
                header=Header(frame_id="map"),
                pose=PoseWithCovariance(pose=pose),
                twist=TwistWithCovariance(
                    twist=Twist(
                        linear=Vector3(*current_vel),
                        angular=Vector3(*current_angvel),
                    ),
                ),
            ),
        )

    rospy.Timer(rospy.Duration(dt), updateCurrent)

    server.applyChanges()
    rospy.spin()
