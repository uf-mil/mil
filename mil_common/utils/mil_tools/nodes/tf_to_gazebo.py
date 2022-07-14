#!/usr/bin/env python3
import rospy
import tf


def main():
    rospy.init_node("tf_to_gazebo")
    do_cam_fix = rospy.get_param("~cam_fix", False)
    tf_parent = rospy.get_param("~tf_parent", "/measurement")
    listener = tf.TransformListener()
    rate = rospy.Rate(10.0)
    cam_fix_quat = (-0.5, 0.5, -0.5, -0.5)

    while not rospy.is_shutdown():
        try:
            print("============= TRANSFORMS ================")
            for frame_id in listener.getFrameStrings():
                (trans, rot) = listener.lookupTransform(
                    tf_parent, frame_id, rospy.Time(0)
                )
                print("--")
                print(f"Transform {tf_parent} {frame_id}")
                if do_cam_fix:
                    rot = tf.transformations.quaternion_multiply(rot, cam_fix_quat)
                print(
                    "(qx={}, qy={} , qz={}, qw={})".format(
                        rot[0], rot[1], rot[2], rot[3]
                    )
                )
                print(f"(x={trans[0]}, y={trans[1]}, z={trans[2]})")
                euler = tf.transformations.euler_from_quaternion(rot)
                print(f"(Roll={euler[0]}, Pitch={euler[1]}, Yaw={euler[2]})")
                print(
                    "<pose> {} {} {} {} {} {} </pose>".format(
                        trans[0], trans[1], trans[2], euler[0], euler[1], euler[2]
                    )
                )
        except (
            tf.LookupException,
            tf.ConnectivityException,
            tf.ExtrapolationException,
        ):
            continue
        rate.sleep()


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print
