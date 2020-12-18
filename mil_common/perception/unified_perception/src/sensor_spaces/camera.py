#!/usr/bin/env python
'''
This is the code required to describe a camera sensor space for unified perception
This uses the pinhole camera model for all math
'''
import numpy as np
import rospy
import tf
from rospy.numpy_msg import numpy_msg
from sensor_space import SensorSpace
from mil_msgs.msg import GaussianDistributionStamped, GaussianDistribution2DStamped
from sensor_msgs.msg import Image, CameraInfo
from image_geometry import PinholeCameraModel
from threading import Lock
from cv_bridge import CvBridge
import cv2
from scipy.stats import multivariate_normal

from matplotlib import pyplot as plt

from distributions.gaussian import Gaussian


class Camera(SensorSpace):
  def __init__(self, name):
    topic = rospy.get_param(name+'/image_topic')
    super(Camera, self).__init__(name, topic, Image)
    # subscribe to the camera topic as well as the camera info topic
    self.info_topic = rospy.get_param(self.name+'/info_topic')
    self.info = rospy.wait_for_message(self.info_topic, CameraInfo)
    self.model = PinholeCameraModel()
    self.model.fromCameraInfo(self.info)
    o_frame = self.model.tfFrame()
    self.optical_frame = o_frame[o_frame.rfind('/')+1:]
    self.listener.waitForTransform(self.optical_frame, self.world_frame, rospy.Time(0),
                                   rospy.Duration(10))
    self.cv_bridge = CvBridge()
    self.score_scale = rospy.get_param(self.name + '/score_scale')
    self.cov_scale = rospy.get_param(self.name + '/cov_scale')

  def project_to_world_frame(self, msg):
    '''msg is a Gaussiandistribution or a GaussianDistribution2D Stamped
    '''
    #TODO add projection from 2D
    if isinstance(msg, GaussianDistributionStamped):
      g = Gaussian(len(msg.distribution.mu), msg.distribution.mu,
                   msg.distribution.cov, msg.header.frame_id,
                   classification=msg.distribution.classification)
      g_wf, t, q = g.transform_to_frame(self.world_frame, self.listener, msg.header.stamp)
      return g_wf
    else:
      raise Exception(self.name + ' was given ' + str(type(msg)) + ', which is not supported')

  def apply_score_to(self, m, new_msgs, idx, score, min_score):
    '''
      msg is a message from the msg_buffer
      mutex is msg's associated mutex
      score is a the output of applying a Gaussian to a msg
      min_score is the cutoff for when the score is so low, that the data there should be removed
      (in this case it means blacked out
    '''
    img = self.cv_bridge.imgmsg_to_cv2(m, desired_encoding='passthrough')

    new_img = img.copy()
    for i in xrange(3):
      new_img[:,:,i] = np.multiply(img[:,:,i],
                        np.clip((score*self.score_scale).astype(np.float), 0, 1))


    new_msg = self.cv_bridge.cv2_to_imgmsg(new_img)
    new_msg.header = m.header
    new_msg.height = m.height
    new_msg.width = m.width
    new_msg.encoding = m.encoding
    new_msg.is_bigendian = m.is_bigendian
    new_msg.step = m.step
    new_msgs[idx] = new_msg
    return


  def project_from_world_frame(self, dist):
    '''
      dist is a Gaussian that is in the world frame and needs to be projected onto the sensor
      (in this case onto and image)
    '''
    # TODO: test
    # NOTE: gaussian distributions are the only ones that are supported right now
    g = dist.transform_to_frame(self.optical_frame, self.listener, time=rospy.Time.now())[0]
    if g.mu[2] < 0:
      return
    # project mu onto the camera pixel coordinates
    mu_pix = np.array(self.model.project3dToPixel(g.mu))
    # project cov from optical frame onto the 2d camera space
    #   https://stackoverflow.com/questions/49239473/how-to-project-3d-covariance-matrix-to-a-given-image-plane-pose
    X_c = g.mu[0]
    Y_c = g.mu[1]
    Z_c = g.mu[2]
    J_f_intr = self.model.intrinsicMatrix()[0:2, 0:2]
    J_f_persp = np.array([[1/Z_c, 0,     -X_c/Z_c**2],
                          [0,     1/Z_c, -Y_c/Z_c**2]])
    R_cw = self.model.rotationMatrix()
    J_f = J_f_intr * J_f_persp * R_cw
    cov_pix = J_f * g.cov * J_f.T
    cov_pix = np.abs(cov_pix) * self.cov_scale
    # 2D gaussian dist which can be multiplied with other distributions returned by this function
    # return the projected distrubution
    return Gaussian(2, mu_pix, cov_pix, self.optical_frame)


  def evaluate_distributions(self, dists):
    height = self.info.height
    width = self.info.width
    score = np.zeros((height, width))
    x, y = np.mgrid[0:height:1, 0:width:1]
    pos = np.dstack((y,x))
    for dist in dists.values():
      score += dist.pdf(pos)
    return score;
