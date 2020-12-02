#!/usr/bin/env python
'''
This is the code required to describe a camera sensor space for unified perception
/lo
This uses the pinhole camera model for all math
'''
import numpy as np

import rospy
import tf
from rospy.numpy_msg import numpy_msg
from sensor_space import SensorSpace
from mil_msgs.srv import GaussianDistribution
from sensor_msgs.msg import Image, CameraInfo
from image_geometry import PinholeCameraModel

class Camera(SensorSpace):

  def __init__(self, topic):
    super(Camera, self).__init__(topic)
    # subscribe to the camera topic as well as the camera info topic
    self.sub = rospy.Subscriber(self.topic, numpy_msg(Image), self.callback)
    # not sure if this will work
    # TODO get info topic from rosparam
    info_topic = self.topic[:self.topic.rfind('/')] + '/info'
    self.info = rospy.wait_for_message(self.info_topic, CameraInfo)
    self.model = PinholeCameraModel()
    self.model.fromCameraInfo(self.info)
    self.score = np.zeros((self.info.height, self.info.width, 3), np.uint8)


  def extract_interesting_region(self, (msg, mutex), score, min_score):
    #TODO
    # lock the mutex
    mutex.lock()
    # make every pixel that is below the min_score black
    # unlock the mutex
    mutex.unlock()
    return


  def project_distribution_from(self, dist):
    #TODO
    # poject from that frame into the camera's optical frame
    # get the projection matrices for the nessesary transform
    try:
      (trans, quat) = self.listener.lookupTransform(self.info.header.frame_id,
                                                   dist.header.frame_id,
                                                   dist.header.stamp)
    except(tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
      continue
    # transform the dist into the optical frame
    # NOTE: gaussian distributions are the only ones that are supported right now
    if dist is GaussianDistribution:
      cov = dist.cov.reshape((3,3))
      # transform the mu and convariance to the optical frame via
      # transpose the mu
      mu = dist.mu + trans
      # math reference: https://robotics.stackexchange.com/questions/2556/how-to-rotate-covariance
      R = tf.transforms.quaterion_matrix(quat)
      sig = np.matmul(R, np.matmul(cov, R.T))
      # project mu onto the camera pixel coordinates
      mu_pix = self.model.project3dtoPixel(mu)
      # TODO project from optical frame onto the 2d camera space
      #   https://stackoverflow.com/questions/49239473/how-to-project-3d-covariance-matrix-to-a-given-image-plane-pose
      X_c = mu[0]
      Y_c = mu[1]
      Z_c = mu[2]
      J_f = np.matmul(np.matmul(self.model.intrinsicMatrix[0:2, 0:2],
                                np.array([[1/Z_c, 0,     -X_c/Z_c**2],
                                          [0,     1/Z_c, -Y_c/Z_c**2]])),
                      self.model.rotationMatrix)
      cov_pix = np.matmul(J_f, np.matmul(cov, J_f.T))
      # 2D gaussian dist which can be multiplied with other distributions returned by this function
      
      # return the projected distrubution
    return


  def reset_score(self):
    score[0].fill(0)


  def add_distribution(self, distribution):
    #TODO
    # evaluate the distribution on the range of the image
    # add that the the score[0]
    return


  def callback(msg):
    msg_buffer.append((msg, Lock()))
    return
