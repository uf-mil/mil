#!/usr/bin/env python3
"""
Usage: rosrun mil_dev_tools video_player _filename:=MyVideoFile.mp4
       rosrun mil_dev_tools video_player MyVideoFile.mp4

Utility to play video files into a ros topic
with some added conveniences like pausing and
scrolling through the video with a slider bar.

Plays any videofile that OpenCV can open (mp4,etc)
Publishes to video_player/filename/image_raw

Set the following rosparams for customization
~filename     string    what file to load
~slider       boolean   True to open window with slider bar and pause
~start_frames int       number of frames into video to start playback at

If slider is set to True (default), a window will open with a
slider bar for moving through the video. Press space in this window
to pause the video and update the slider position. While paused,
you can press s to go frame by frame.

"""
import os
import sys

import cv2
import rclpy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image


class RosVideoPlayer:
    def __init__(self):
        self.bridge = CvBridge()
        self.filename = self.declare_parameter("~filename", sys.argv[1])
        image_topic = (
            "video_player/"
            + os.path.splitext(os.path.basename(self.filename))[0]
            + "/image_raw",
        )
        self.image_pub = self.create_publisher(Image, image_topic, 10)
        self.slider = self.declare_parameter("~slider", True)
        self.start_frame = self.declare_parameter("~start_frames", 0)
        self.cap = cv2.VideoCapture(self.filename)

        self.width = self.cap.get(cv2.cv.CV_CAP_PROP_FRAME_WIDTH)
        self.height = self.cap.get(cv2.cv.CV_CAP_PROP_FRAME_HEIGHT)

        self.roi_y_offset = self.declare_parameter("~y_offset", 0)
        self.roi_x_offset = self.declare_parameter("~x_offset", 0)
        self.roi_height = self.declare_parameter("~height", self.height)
        self.roi_width = self.declare_parameter("~width", self.width)

        self.num_frames = self.cap.get(cv2.cv.CV_CAP_PROP_FRAME_COUNT)
        self.fps = self.declare_parameter("~fps", self.cap.get(cv2.cv.CV_CAP_PROP_FPS))
        self.cap.set(cv2.cv.CV_CAP_PROP_POS_FRAMES, self.start_frame)
        self.num_frames = self.cap.get(cv2.cv.CV_CAP_PROP_FRAME_COUNT)
        if self.num_frames < 1:
            raise Exception(f"Cannot read video {self.filename}")

        self.paused = False
        self.ended = False
        self.last_key = 255

        if self.slider:
            cv2.namedWindow("Video Control")
            cv2.createTrackbar(
                "Frame",
                "Video Control",
                int(self.start_frame),
                int(self.num_frames),
                self.trackbar_cb,
            )
            cv2.createTrackbar(
                "ROI x_offset",
                "Video Control",
                int(self.roi_x_offset),
                int(self.width) - 1,
                self.roi_x_cb,
            )
            cv2.createTrackbar(
                "ROI y_offset",
                "Video Control",
                int(self.roi_y_offset),
                int(self.height) - 1,
                self.roi_y_cb,
            )
            cv2.createTrackbar(
                "ROI height",
                "Video Control",
                int(self.roi_height),
                int(self.height),
                self.roi_height_cb,
            )
            cv2.createTrackbar(
                "ROI width",
                "Video Control",
                int(self.roi_width),
                int(self.width),
                self.roi_width_cb,
            )
        self.get_logger().info(
            "Playing {} at {}fps starting at frame {} ({} Total Frames)".format(
                self.filename,
                self.fps,
                self.start_frame,
                self.num_frames,
            ),
        )

    def run(self):
        r = rclpy.Rate(self.fps)  # 10hz
        while not rclpy.is_shutdown() and self.cap.isOpened() and not self.ended:
            if self.slider:
                k = cv2.waitKey(1) & 0xFF
                if k != self.last_key:
                    self.last_key = k
                    if k == 27:
                        return
                    elif k == 32:
                        self.pause()
                    elif k == 115 and self.paused:
                        self.one_frame()
            if self.paused:
                cv2.setTrackbarPos(
                    "Frame",
                    "Video Control",
                    int(self.cap.get(cv2.cv.CV_CAP_PROP_POS_FRAMES)),
                )
            else:
                self.one_frame()
            r.sleep()

    def one_frame(self):
        ret, frame = (None, None)
        try:
            ret, frame = self.cap.read()
        except Exception as e:
            print(f"Exception: {e}")
        if not ret:
            if not self.ended:
                self.get_logger().info(f"File {self.filename} ended")
            self.ended = True
            return
        else:
            self.ended = False
            x1 = self.roi_x_offset
            x2 = x1 + self.roi_width
            y1 = self.roi_y_offset
            y2 = y1 + self.roi_height
            frame_roied = frame[
                y1 : min(y2, frame.shape[0]),
                x1 : min(x2, frame.shape[1]),
            ]
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(frame_roied, "bgr8"))

    def trackbar_cb(self, x):
        self.cap.set(cv2.cv.CV_CAP_PROP_POS_FRAMES, x)

    def roi_x_cb(self, d):
        self.roi_x_offset = d

    def roi_y_cb(self, d):
        self.roi_y_offset = d

    def roi_height_cb(self, d):
        self.roi_height = d if d > 0 else 1

    def roi_width_cb(self, d):
        self.roi_width = d if d > 0 else 1

    def pause(self):
        self.paused = not self.paused


def main():
    rclpy.init(args=sys.argv)

    player = RosVideoPlayer()
    player.run()


if __name__ == "__main__":
    main()
