import argparse
import sys

import cv2
import mil_ros_tools
import numpy as np
import rospy
from boost_auto import observe


def got_image(img):
    global last_image, last_image_time
    last_image = np.copy(img)
    last_image_time = rospy.Time.now()


def process_image(img, image_pub, clf):
    print("Got image!")

    some_observations = observe(img)
    print("-------------------------")
    segmentation = np.array(
        [clf.predict(obs) for obs in some_observations],
    )
    segmentation_image = np.reshape(segmentation, img[:, :, 2].shape)

    image_pub.publish(np.dstack([(segmentation_image * 250).astype(np.uint8)] * 3))
    print("Pubbing Image")


if __name__ == "__main__":
    rospy.init_node("Buoy_Test")

    parser = argparse.ArgumentParser(usage="", description="")
    parser.add_argument(
        dest="classifier",
        type=str,
        help="Name of the classifier to use.",
    )
    parser.add_argument(
        dest="topic",
        type=str,
        help="Topic to listen to for image callbacks",
        default="/camera/front/left/image_rect_color",
    )

    args = parser.parse_args(sys.argv[1:])

    clf = cv2.Boost()
    clf.load(args.classifier)

    image_sub = mil_ros_tools.Image_Subscriber(args.topic, got_image, queue_size=1)
    image_pub = mil_ros_tools.Image_Publisher(args.topic + "_segmented")

    last_image = None
    last_image_time = None
    while not rospy.is_shutdown():
        if last_image is not None and last_image_time is not None:
            process_image(np.copy(last_image), image_pub, clf)
            print((rospy.Time.now() - last_image_time).to_sec())
            print
        rospy.sleep(0.1)

    rospy.spin()
