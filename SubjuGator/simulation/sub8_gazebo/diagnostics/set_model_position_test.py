#!/usr/bin/env python3

import asyncio

import job_runner
import numpy as np
import tf
import txros
import uvloop
from mil_ros_tools import msg_helpers
from nav_msgs.msg import Odometry
from sub8_gazebo.srv import RunJobResponse


async def run_mission(srv, sub, nh):
    print("Running Mission")
    # Generate point to go to
    point = np.random.uniform(-20, 20, size=3)
    point[2] = -np.abs(point[2]) / 4

    quat = tf.transformations.quaternion_from_euler(
        0, 0, np.random.uniform(-7, 7, size=1)
    )

    pose = msg_helpers.numpy_quat_pair_to_pose(point, quat)
    print(pose)

    # Go to point
    await job_runner.JobManager.set_model_position(nh, pose)
    await txros.util.wall_sleep(5.0)
    print("Movement should be complete.")

    excpeted_position = np.array([point, quat])

    actual_odom = msg_helpers.odometry_to_numpy((yield sub.get_next_message()))
    actual_position = np.array(actual_odom[0])

    print(excpeted_position, actual_position)
    print("Done, err:")
    print(np.abs(excpeted_position - actual_position))

    if (np.abs(excpeted_position[0] - actual_position[0]) < 0.5).all() and (
        np.abs(excpeted_position[1] - actual_position[1]) < 0.2
    ).all():
        print("PASS")
        return RunJobResponse(success=True, message=str(pose))
    else:
        print("FAIL")
        err_msg = str(excpeted_position) + "\n" + str(actual_position)
        return RunJobResponse(success=False, message=err_msg)


async def main():
    nh = await txros.NodeHandle.from_argv("test")
    world_position_actual = nh.subscribe("/world_odom", Odometry)
    await world_position_actual.setup()
    run_mission(None, world_position_actual, nh)
    print("Waiting for service call.")


if __name__ == "__main__":
    uvloop.install()
    asyncio.run(main())
