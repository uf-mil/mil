#!/usr/bin/python3
from __future__ import division

import asyncio
import math
import operator
import time
import traceback

import cv2
import numpy
import sensor_msgs.point_cloud2 as pc2
import txros
import uvloop
from geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion, Vector3
from mil_tools import msg_helpers
from nav_msgs.msg import Odometry
from navigator_msgs.srv import ObjectDBQuery, ObjectDBQueryRequest
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import ColorRGBA, Header
from tf import transformations
from txros import txros_tf, util
from visualization_msgs.msg import Marker, MarkerArray


def rect_lines(rect):
    for i in range(4):
        angle = rect[4] + i / 4 * 2 * math.pi
        normal = numpy.array([math.cos(angle), math.sin(angle)])
        d_normal_over_d_angle = numpy.array([-math.sin(angle), math.cos(angle)])
        p = rect[:2] + (rect[2] if i % 2 == 0 else rect[3]) / 2 * normal
        yield (p, normal), numpy.array(
            [
                [
                    1,
                    0,
                    normal[0] / 2 if i % 2 == 0 else 0,
                    normal[0] / 2 if i % 2 == 1 else 0,
                    (rect[2] if i % 2 == 0 else rect[3]) / 2 * d_normal_over_d_angle[0],
                ],
                [
                    0,
                    1,
                    normal[1] / 2 if i % 2 == 0 else 0,
                    normal[1] / 2 if i % 2 == 1 else 0,
                    (rect[2] if i % 2 == 0 else rect[3]) / 2 * d_normal_over_d_angle[1],
                ],
                [0, 0, 0, 0, d_normal_over_d_angle[0]],
                [0, 0, 0, 0, d_normal_over_d_angle[1]],
            ]
        )


def dist_from_line(p, line, J):
    return (p - line[0]).dot(line[1]), numpy.array(
        [-line[1][0], -line[1][1], p[0] - line[0][0], p[1] - line[0][1]]
    ).dot(J)


def dist_from_rect(p, rect):
    return min(
        (dist_from_line(p, line, J) for line, J in rect_lines(rect)),
        key=lambda dist_J: abs(dist_J[0]),
    )


def fit(good_points, rect):
    print("count", len(good_points))
    for i in range(1):
        r, J = zip(*[dist_from_rect(pnt, rect) for pnt in good_points])
        r = numpy.array(r)
        J = numpy.vstack(list(J))
        cost = numpy.sum(r * r)
        print("cost", i, cost)
        try:
            rect = rect - numpy.linalg.inv(J.T.dot(J)).dot(J.T).dot(r)
        except numpy.linalg.LinAlgError:
            traceback.print_exc()
            return None
        # print rect
    return rect


center_holder = [None]


async def poller(nh):
    db_srv = nh.get_service_client("/database/requests", ObjectDBQuery)

    while True:
        await util.wall_sleep(3)

        while True:
            try:
                db_res = await db_srv(
                    ObjectDBQueryRequest(
                        name="shooter",
                        cmd="",
                    )
                )
            except BaseException:
                traceback.print_exc()
                await util.wall_sleep(1)
            else:
                break

        if not db_res.found:
            print("shooter not found")
            continue

        obj = db_res.objects[0]

        points = map(msg_helpers.ros_to_np_3D, obj.points)

        if points:
            center_holder[0] = numpy.mean(points, axis=0)


async def main():
    try:
        center = None
        nh = txros.NodeHandle.from_argv("shooter_finder", anonymous=True)
        await nh.setup()

        pc_sub = nh.subscribe("/velodyne_points", PointCloud2)
        tf_listener = txros_tf.TransformListener(nh)
        marker_pub = nh.advertise("/shooter_fit", MarkerArray)
        result_pub = nh.advertise("/shooter_pose", PoseStamped)
        odom_sub = nh.subscribe("/odom", Odometry)

        await asyncio.gather(
            pc_sub.setup(),
            tf_listener.setup(),
            marker_pub.setup(),
            result_pub.setup(),
            odom_sub.setup(),
        )

        await poller(nh)

        while True:
            ts = [time.time()]
            await util.wall_sleep(0.1)
            ts.append(time.time())

            center = center_holder[0]
            odom = await odom_sub.get_next_message()
            cloud = await pc_sub.get_next_message()

            if center is None:
                await util.wall_sleep(1)
                continue

            # print center

            Z_MIN = 0
            RADIUS = 5

            ts.append(time.time())

            print("start")
            print(cloud.header.stamp)
            try:
                transform = await tf_listener.get_transform(
                    "/enu", cloud.header.frame_id, cloud.header.stamp
                )
            except txros_tf.TooPastError:
                print("TooPastError!")
                continue
            gen = numpy.array(
                list(
                    pc2.read_points(cloud, skip_nans=True, field_names=("x", "y", "z"))
                )
            ).T
            print(gen.shape)
            print(transform._p)
            gen = (
                transform._q_mat.dot(gen)
                + numpy.vstack([transform._p] * gen.shape[1]).T
            ).T
            good_points = numpy.array(
                [
                    (x[0], x[1])
                    for x in gen
                    if x[2] > Z_MIN
                    and math.hypot(x[0] - center[0], x[1] - center[1]) < RADIUS
                ]
            )
            print("end")

            if len(good_points) < 10:
                print("no good points", len(good_points))
                continue

            # if len(good_points) > 100:
            #    good_points = numpy.array(random.sample(list(good_points), 100))
            # print good_points

            ts.append(time.time())

            rect = cv2.minAreaRect(good_points.astype(numpy.float32))
            rect = numpy.array(rect[0] + rect[1] + (math.radians(rect[2]),))

            ts.append(time.time())

            # rect = await threads.deferToThread(fit, good_points, rect)
            if rect is None:
                print("bad fit")
                continue

            rect = rect[:2], rect[2:4], rect[4]

            print("rect:", rect)

            ts.append(time.time())

            marker_pub.publish(
                MarkerArray(
                    markers=[
                        Marker(
                            header=Header(
                                frame_id="/enu",
                                stamp=nh.get_time(),
                            ),
                            ns="shooter_ns",
                            id=1,
                            type=Marker.CUBE,
                            action=Marker.ADD,
                            pose=Pose(
                                position=Point(rect[0][0], rect[0][1], 0.5),
                                orientation=Quaternion(
                                    *transformations.quaternion_about_axis(
                                        rect[2], [0, 0, 1]
                                    )
                                ),
                            ),
                            scale=Vector3(rect[1][0], rect[1][1], 2),
                            color=ColorRGBA(1, 1, 1, 0.5),
                        )
                    ],
                )
            )

            possibilities = []
            for i in range(4):
                angle = rect[2] + i / 4 * 2 * math.pi
                normal = numpy.array([math.cos(angle), math.sin(angle)])
                p = (
                    numpy.array(rect[0])
                    + (rect[1][0] if i % 2 == 0 else rect[1][1]) / 2 * normal
                )
                possibilities.append(
                    (
                        normal,
                        p,
                        transformations.quaternion_about_axis(angle, [0, 0, 1]),
                        rect[1][0] if i % 2 == 1 else rect[1][1],
                    )
                )
            best = max(
                possibilities,
                key=lambda normal_p_q_size: (
                    msg_helpers.ros_to_np_3D(odom.pose.pose.position)[:2] - rect[0]
                ).dot(normal_p_q_size[0]),
            )

            print("side length:", best[-1])
            if abs(best[-1] - 1.8) > 0.25:
                print("filtered out based on side length")
                continue

            front_points = [
                pnt for pnt in good_points if abs((pnt - best[1]).dot(best[0])) < 0.2
            ]
            print("len(front_points)", len(front_points))
            a, b = numpy.linalg.lstsq(
                numpy.vstack(
                    [
                        [pnt[0] for pnt in front_points],
                        [pnt[1] for pnt in front_points],
                    ]
                ).T,
                numpy.ones(len(front_points)),
            )[0]
            m, b_ = numpy.linalg.lstsq(
                numpy.vstack(
                    [
                        [pnt[0] for pnt in front_points],
                        numpy.ones(len(front_points)),
                    ]
                ).T,
                [pnt[1] for pnt in front_points],
            )[0]
            print("a, b", a, b, m, b_)

            print(
                "RES", numpy.std([numpy.array([a, b]).dot(pnt) for pnt in good_points])
            )

            # x = a, b
            # x . p = 1
            # |x| (||x|| . p) = 1
            # ||x|| . p = 1 / |x|

            normal = numpy.array([a, b])
            dist = 1 / numpy.linalg.norm(normal)
            normal = normal / numpy.linalg.norm(normal)
            p = dist * normal

            print("ZZZ", p.dot(numpy.array([a, b])))

            perp = numpy.array([normal[1], -normal[0]])
            x = (best[1] - p).dot(perp)
            p = p + x * perp
            if normal.dot(best[0]) < 0:
                normal = -normal
            q = transformations.quaternion_about_axis(
                math.atan2(normal[1], normal[0]), [0, 0, 1]
            )

            print("XXX", p, best[1])

            # sqrt(a2 + b2) (a x + b y) = 1

            result_pub.publish(
                PoseStamped(
                    header=Header(
                        frame_id="/enu",
                        stamp=nh.get_time(),
                    ),
                    pose=Pose(
                        position=Point(p[0], p[1], 0),
                        orientation=Quaternion(*q),
                    ),
                )
            )

            ts.append(time.time())

            print("timing", ts[-1] - ts[0], map(operator.sub, ts[1:], ts[:-1]))
    except BaseException:
        traceback.print_exc()


if __name__ == "__main__":
    uvloop.install()
    asyncio.run(main())
