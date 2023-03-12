#!/usr/bin/env python3

import math

import numpy
import roslib
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
from mil_msgs.msg import PoseTwist
from rise_6dof.controller import Controller
from tf import transformations

if __name__ == "__main__":
    roslib.load_manifest("rise_6dof")

    print(transformations.quaternion_from_euler(math.pi / 2, 0, math.pi / 2))

    c = Controller(
        dict(
            k=numpy.ones(6),
            ks=numpy.ones(6),
            alpha=numpy.ones(6),
            beta=numpy.ones(6),
            use_rise=False,
        ),
    )

    desired = PoseTwist(
        pose=Pose(
            position=Point(
                x=1,
                y=2,
                z=3,
            ),
            orientation=Quaternion(
                x=0.5,
                y=0.5,
                z=0.5,
                w=0.5,
            ),
        ),
        twist=Twist(
            linear=Vector3(
                x=0,
                y=0,
                z=0,
            ),
            angular=Vector3(
                x=0,
                y=0,
                z=1,
            ),
        ),
    )
    current = PoseTwist(
        pose=Pose(
            position=Point(
                x=1,
                y=2,
                z=3,
            ),
            orientation=Quaternion(
                x=0.5,
                y=0.5,
                z=0.5,
                w=0.5,
            ),
        ),
        twist=Twist(
            linear=Vector3(
                x=0,
                y=0,
                z=0,
            ),
            angular=Vector3(
                x=0,
                y=0,
                z=0,
            ),
        ),
    )

    print(c.update(0.01, desired, current))
