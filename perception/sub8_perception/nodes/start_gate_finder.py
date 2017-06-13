#!/usr/bin/env python
import cv2
import numpy as np
import rospy
import tf
from mil_ros_tools import numpy_to_point, Image_Publisher, Image_Subscriber, numpy_to_quaternion
from image_geometry import PinholeCameraModel
from mil_vision_tools import RectFinder
from geometry_msgs.msg import PointStamped, Point, Vector3Stamped

class StartGateFinder():
	def __init__(self):

		length = 1.0033
		width = 1.9304
		self.rect_model = RectFinder(length, width)

		self.tf_listener = tf.TransformListener()
		self.image_sub = Image_Subscriber('/camera/front/left/image_raw', self._img_callback_a)
		# self.image_sub2 = Image_Subscriber('/camera/front/left/image_raw', self._img_callback_b)

		# self.image_sub = Image_Subscriber('/stereo/left/image_rect_color', self._img_callback_a)
		# self.image_sub2 = Image_Subscriber('/stereo/left/image_rect_color', self._img_callback_b)
		# self.camera_info = self.image_sub.wait_for_camera_info()
		# assert self.camera_info is not None
		# self.cam = PinholeCameraModel()
		# self.cam.fromCameraInfo(self.camera_info)

		self.center_a = 0
		self.center_b = 0
		self.last_bgr = None


	def _get_angle(self, a, b, c):
		ba = a[0]-b[0]
		bc = c[0]-b[0]
		cos_ang = np.dot(ba, bc) / (np.linalg.norm(ba) * np.linalg.norm(bc))
		angle = np.arccos(cos_ang)
		return np.degrees(angle)

	def _valid_contour(self, contour):
		area = cv2.contourArea(contour)
		if(area < 1000):
			return False
		# 310408
		if(area > 30000):
			return False

		epsilon = 0.01*cv2.arcLength(contour,True)
		approx = cv2.approxPolyDP(contour,epsilon,True)
		rect = cv2.minAreaRect(approx)
		if(rect[1][0]*rect[1][1] < 7000):
			return False;


		if(len(approx) < 8):
			return False
		if(len(approx) > 13):
			return False;
		# cv2.drawContours(self.last_bgr, approx, -1, (0, 255, 255), 5)
		for i in range(0, len(approx)-2, 2):
			angle = self._get_angle(approx[i], approx[i+1], approx[i+2])
			if(np.abs(angle - 90) > 23):
				return False
		angle = self._get_angle(approx[len(approx)-1], approx[0], approx[1])
		if(np.abs(angle - 90) > 23):
			return False

		return True


	def _get_edges(self, image):
		blur = cv2.blur(image, (5, 5))
		# _, blur = cv2.threshold(blur,140,255,cv2.THRESH_TRUNC)
		# cv2.imshow("blur", blur)
		# cv2.imshow("img", self.last_lab)
		# cv2.imshow("test", cv2.Canny(self.last_lab, 20, 20 * 3.0))
		cv2.imshow("imag", blur)
		# 
		kernel = np.ones((5,5),np.uint8)
		canny = cv2.Canny(blur, 30, 30 * 3.0)

		cv2.imshow("canny", canny)
		closing = cv2.morphologyEx(canny, cv2.MORPH_CLOSE, kernel)
		# cv2.imshow("closing", closing)
		dilation = cv2.dilate(closing,kernel,iterations = 2)
		# cv2.imshow("dial", dilation)
		return dilation

	def _img_callback_a(self, img):
		self.last_bgr = img;
		# Pool/red thing:
		image = cv2.cvtColor(img, cv2.COLOR_BGR2LAB)
		image_lab_a = image[:,:,1]
		image_hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
		image_hsv_s = image_hsv[:,:,0]
		image = cv2.bitwise_and(image_lab_a, image_hsv_s)

		#Transdeck /yellow thing:
		# image = cv2.cvtColor(img, cv2.COLOR_BGR2LAB)

		edges = self._get_edges(image)
		_, contours, _ = cv2.findContours(edges, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
		for idx, c in enumerate(contours):
			if(self._valid_contour(c)):
				cv2.drawContours(self.last_bgr, contours, idx, (200, 0, 0), 3)
				M = cv2.moments(c)
				cX = int(M["m10"] / M["m00"])
				cY = int(M["m01"] / M["m00"])
				self.center_a = (cX, cY)
				cv2.circle(img, (cX, cY), 3, (200, 0, 0), -1)
				# cv2.drawContours(self.last_bgr,[box],0,(0,0,255),2)
		

		cv2.imshow("rect", img)
		print self.center_a, self.center_b
		# cv2.imshow("debug", img)
		cv2.waitKey(30)

if __name__ == '__main__':
    rospy.init_node('start_gate_finder')
    StartGateFinder()
    rospy.spin()
