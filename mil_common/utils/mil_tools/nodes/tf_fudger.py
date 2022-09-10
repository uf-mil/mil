#!/usr/bin/env python3

import argparse
import sys

import cv2
import numpy as np
import rospy
import tf
import tf.transformations as trns

rospy.init_node("tf_fudger", anonymous=True)
br = tf.TransformBroadcaster()

usage_msg = "Useful to test transforms between two frames."
desc_msg = "Pass the name of the parent frame and the child name as well as a an optional multiplier. \n\
At anypoint you can press q to switch between displaying a quaternion and an euler angle. \n\
Press 's' to exit and REPLACE or ADD the line in the tf launch file. Use control-c if you don't want to save."

parser = argparse.ArgumentParser(usage=usage_msg, description=desc_msg)
parser.add_argument(dest="tf_parent", help="Parent tf frame.")
parser.add_argument(dest="tf_child", help="Child tf frame.")
parser.add_argument(
    "--range",
    action="store",
    type=float,
    default=3,
    help="Maximum range allowed for linear displacement (in meters)",
)

args = parser.parse_args(sys.argv[1:])

cv2.namedWindow("tf", cv2.WINDOW_NORMAL)

# slider max values
x_max = 1000
ang_max = 3000

# tf ranges (symmetric about zero)
x_range = args.range  # meters
ang_range = 2 * np.pi  # radians

# tf fudging resolution
x_res = x_range / x_max
ang_res = ang_range / ang_max

print(
    "Distance resolution: {} meters\nAngular resolution: {} radians".format(
        x_res, ang_res
    )
)

cv2.createTrackbar("x", "tf", 0, x_max, lambda x: x)
cv2.createTrackbar("y", "tf", 0, x_max, lambda x: x)
cv2.createTrackbar("z", "tf", 0, x_max, lambda x: x)

cv2.createTrackbar("roll", "tf", 0, ang_max, lambda x: x)
cv2.createTrackbar("pitch", "tf", 0, ang_max, lambda x: x)
cv2.createTrackbar("yaw", "tf", 0, ang_max, lambda x: x)


def toTfLin(x):
    return x * x_res - 0.5 * x_range


def toTfAng(x):
    return x * ang_res - 0.5 * ang_range


def toCvLin(x):
    return (x + 0.5 * x_range) / x_res


def toCvAng(x):
    return (x + 0.5 * ang_range) / ang_res


p = q = None
q_mode = False

# Used for printing and saving
tf_line = '<node pkg="tf" type="static_transform_publisher" \
          name="{child}_tf" args="{p[0]} {p[1]} {p[2]}\
          {q[0]} {q[1]} {q[2]} {q[3]}  /{parent} /{child} {prd}" />\n'
prd = 100  # This will get replaced if it needs to

args.tf_child = args.tf_child[1:] if args.tf_child[0] == "/" else args.tf_child
args.tf_parent = args.tf_parent[1:] if args.tf_parent[0] == "/" else args.tf_parent

listener = tf.TransformListener()

p_original = (0, 0, 0)
rpy_original = (0, 0, 0)
try:
    listener.waitForTransform(
        args.tf_parent, args.tf_child, rospy.Time(0), rospy.Duration(1)
    )
    (trans, rot) = listener.lookupTransform(
        args.tf_parent, args.tf_child, rospy.Time(0)
    )
    euler = trns.euler_from_quaternion(rot)
    p_original = trans
    rpy_original = euler
except tf.Exception:
    print("TF not found, setting everything to zero")


def set_bars(p, rpy):
    cv2.setTrackbarPos("x", "tf", int(toCvLin(p[0])))
    cv2.setTrackbarPos("y", "tf", int(toCvLin(p[1])))
    cv2.setTrackbarPos("z", "tf", int(toCvLin(p[2])))
    cv2.setTrackbarPos("roll", "tf", int(toCvAng(rpy[0])))
    cv2.setTrackbarPos("pitch", "tf", int(toCvAng(rpy[1])))
    cv2.setTrackbarPos("yaw", "tf", int(toCvAng(rpy[2])))
    cv2.waitKey(100) & 0xFF


def reset():
    set_bars(p_original, rpy_original)


reset()

p_last = p_original
rpy_last = rpy_original

while not rospy.is_shutdown():
    x, y, z = (
        toTfLin(cv2.getTrackbarPos("x", "tf")),
        toTfLin(cv2.getTrackbarPos("y", "tf")),
        toTfLin(cv2.getTrackbarPos("z", "tf")),
    )
    p = (x, y, z)
    rpy = (
        toTfAng(cv2.getTrackbarPos("roll", "tf")),
        toTfAng(cv2.getTrackbarPos("pitch", "tf")),
        toTfAng(cv2.getTrackbarPos("yaw", "tf")),
    )
    q = tf.transformations.quaternion_from_euler(*rpy)

    if (not p == p_last) or (not rpy == rpy_last):
        rpy_feedback = "xyz: {}, euler: {}".format(
            [round(e, 5) for e in p], [round(np.degrees(f), 5) for f in rpy]
        )
        q_feedback = "xyz: {},     q: {}".format(
            [round(x, 5) for e in p], [round(f, 5) for f in q]
        )
        print(q_feedback if q_mode else rpy_feedback)
    p_last = p
    rpy_last = rpy

    k = cv2.waitKey(100) & 0xFF
    if k == ord("q"):
        q_mode = not q_mode

    if k == ord("r"):
        reset()

    if k == ord("z"):
        set_bars((0, 0, 0), (0, 0, 0))

    # This functionality was configured to replace lines in navigator's tf
    # launch file, should refactor to be general later
    """
    if k == ord('s'):
        # Save the transform in navigator_launch/launch/tf.launch replacing the line
        import rospkg
        rospack = rospkg.RosPack()
        launch_path = rospack.get_path('navigator_launch')
        launch_path += "/launch/subsystems/tf.launch"

        with open(launch_path, 'r') as f:
            lines = f.readlines()

        last_static_pub = 0
        tab_level = ''
        np.set_printoptions(suppress=True)
        tf_line_to_add = tf_line.format(child=args.tf_child, p=p, q=np.round(q, 5), parent=args.tf_parent, prd="{prd}")
        for i,line in enumerate(lines):
            if args.tf_child in line and args.tf_parent in line:
                tab_level = line[:line.find("<")]  # The labs infront of the tf line
                prd = int([x for x in line.split('"')[-2].split(" ") if x != ''][-1])  # Get the period from the tf line
                lines[i] = tab_level + tf_line_to_add.format(prd=prd)

                print("Found line in tf.launch!")
                print("Replacing: {}".format(line.replace(tab_level, '')[:-1]))
                print("     With: {}".format(lines[i].replace(tab_level, '')[:-1]))
                break

            elif 'pkg="tf"' in line:
                # In case we don't find the tf line of interest to change, we want to insert the new tf line after
                # the last tf line
                tab_level = line[:line.find("<")]  # The labs infront of the tf line
                last_static_pub = i

        else:
            print("Tf link not found between /{parent} /{child} in tf.launch.".format(
                   parent=args.tf_parent, child=args.tf_child))
            lines.insert(last_static_pub + 1, '\n' + tab_level + tf_line_to_add.format(prd=prd))
            print("Adding: {}".format(lines[last_static_pub + 1].replace(tab_level, '')[:-1]))

        with open(launch_path, 'w') as f:
            for line in lines:
                f.write(line)

        break
    """

    br.sendTransform(p, q, rospy.Time.now(), args.tf_child, args.tf_parent)

# Print out the tf static transform line with the fudged tf
print(
    "\n",
    tf_line.format(
        child=args.tf_child, p=p, q=np.round(q, 5), parent=args.tf_parent, prd=prd
    ),
)
