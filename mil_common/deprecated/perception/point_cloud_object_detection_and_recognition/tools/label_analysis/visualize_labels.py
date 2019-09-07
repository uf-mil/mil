#! /usr/bin/env python

import rosbag
import matplotlib.pyplot as plt

if __name__ == '__main__':
    # TODO(tbianchi): Add argument parser

    xs = []
    ys = []
    zs = []
    bag = rosbag.Bag('/home/tess/mil_ws/labels_10_5_v2.bag')
    c = 0
    for topic, msg, t in bag.read_messages(topics=['labels']):
        print c
        if c == 1:
            break
        for obj in msg.objects:
            if obj.classification == "scan_the_code":
                c += 1
                for p in obj.points:
                    xs.append(p.x)
                    ys.append(p.y)
                    zs.append(p.z)

    bag.close()
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.scatter(xs, ys, zs)
    plt.show()
