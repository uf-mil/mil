#!/usr/bin/env python
"""
Usage: rosrun navigator_tools video_player.py _filename:=MyVideoFile.mp4

Utility to play video files into a ros topic
with some added conveniences like pausing and
scrolling through the video with a slider bar.

Plays any videofile that OpenCV can open (mp4,etc)
Publishes to video_file/image_raw

Set the following rosparams for customization
~filename     string    what file to load
~slider       boolean   True to open window with slider bar and pause
~start_frames int       number of frames into video to start playback at

If slider is set to True (default), a window will open with a
slider bar for moving through the video. Press space in this window
to pause the video and update the slider position. While paused,
you can press s to go frame by frame.

"""

import sys
import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class RosVideoPlayer:

  def __init__(self):
      self.bridge = CvBridge()
      self.image_pub = rospy.Publisher("video_file/image_raw",Image,queue_size=10)
      self.filename =  rospy.get_param('~filename', 'video.mp4')
      self.slider = rospy.get_param('~slider',True)
      self.start_frame = rospy.get_param('~start_frames',0)
      self.cap = cv2.VideoCapture(self.filename)
      self.num_frames = self.cap.get(cv2.cv.CV_CAP_PROP_FRAME_COUNT)
      self.fps = rospy.get_param('~fps',self.cap.get(cv2.cv.CV_CAP_PROP_FPS))
      self.cap.set(cv2.cv.CV_CAP_PROP_POS_FRAMES,self.start_frame)
      self.num_frames = self.cap.get(cv2.cv.CV_CAP_PROP_FRAME_COUNT)
      self.paused = False
      self.ended = False
      self.last_key = 255
      
      if (self.slider):
          cv2.namedWindow('Video Control')
          cv2.createTrackbar('Frame','Video Control',int(0),int(self.num_frames),self.trackbar_cb)
      rospy.loginfo("Playing {} at {}fps starting at frame {} ({} Total Frames)".format(self.filename,self.fps,self.start_frame,self.num_frames))

  def run(self):
      r = rospy.Rate(self.fps) # 10hz
      while (not rospy.is_shutdown()) and self.cap.isOpened():
          if self.slider:
              k = cv2.waitKey(1) & 0xFF
              if not k == self.last_key:
                  self.last_key = k
                  if k == 27:
                      return
                  elif k == 32:
                      self.pause()
                  elif k == 115 and self.paused:
                      self.one_frame()
          if self.paused:
              cv2.setTrackbarPos('Frame','Video Control',int(self.cap.get(cv2.cv.CV_CAP_PROP_POS_FRAMES)))
          else:
              self.one_frame()
          r.sleep()

  def one_frame(self):
      ret, frame = self.cap.read()
      if not ret:
          if not self.ended:
              rospy.loginfo("File {} ended".format(self.filename))
          self.ended = True
          return
      else:
          self.ended = False
          self.image_pub.publish(self.bridge.cv2_to_imgmsg(frame, "bgr8"))    

  def trackbar_cb(self,x):
      self.cap.set(cv2.cv.CV_CAP_PROP_POS_FRAMES,x)

  def pause(self):
      self.paused = not self.paused

def main():
    rospy.init_node('video_player')
    player = RosVideoPlayer()
    player.run()

if __name__ == '__main__':
    main()
