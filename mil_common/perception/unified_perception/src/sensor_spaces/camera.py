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
from mil_msgs.srv import GaussianDistribution, GaussianDistribution2D
from sensor_msgs.msg import Image, CameraInfo
from image_geometry import PinholeCameraModel
from threading import Lock


from distributions.gaussian import Gaussian


class Camera(SensorSpace):

  def __init__(self, name):
    super(Camera, self).__init__(name)
    # TODO get topic from rosparam
    # TODO get info topic from rosparam
    # subscribe to the camera topic as well as the camera info topic
    self.topic = rospy.get_param(self.name+'/image_topic')
    self.sub = rospy.Subscriber(self.topic, numpy_msg(Image), self.callback)
    # not sure if this will work
    self.info_topic = rospy.get_param(self.name+'/info_topic')
    self.info = rospy.wait_for_message(self.info_topic, CameraInfo)
    self.model = PinholeCameraModel()
    self.model.fromCameraInfo(self.info)
    o_frame = self.info.header.frame_id
    self.optical_frame = o_frame[o_frame.rfind('/')+1:]
    self.listener.waitForTransform(self.optical_frame, self.world_frame, rospy.Time(0),
                                   rospy.Duration(10))


  def project_to_world_frame(self, msg):
    '''msg is a Gaussiandistribution or a GaussianDistribution2D
    '''
    g = Gaussian(len(msg.mu), msg.mu, msg.cov, msg.header.frame_id)
    return g.transform_to_frame(self.world_frame, self.listener)[0]


  def apply_distributions_to(self, (msg, mutex), dists, min_score):
    '''
      msg is a message from the msg_buffer
      mutex is msg's associated mutex
      score is a the output of applying a Gaussian to a msg
      min_score is the cutoff for when the score is so low, that the data there should be removed
      (in this case it means blacked out
    '''
    #TODO
    # lock the mutex
    mutex.acquire()
    # make every pixel that is below the min_score black
    # unlock the mutex
    mutex.release()
    return


  def project_from_world_frame(self, dist):
    '''
      dist is a Gaussian that is in the world frame and needs to be projected onto the sensor
      (in this case onto and image)
    '''
    #TODO
    # poject from that frame into the camera's optical frame
    # get the projection matrices for the nessesary transform
    # transform the dist into the optical frame
    # NOTE: gaussian distributions are the only ones that are supported right now
    g = dist.transform_to_frame(self.optical_frame, self.listener, time=rospy.Time.now())[0]
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
    J_f = np.matmul(J_f_intr, np.matmul(J_f_persp, R_cw))
    cov_pix = np.matmul(J_f, np.matmul(g.cov, J_f.T))
    # 2D gaussian dist which can be multiplied with other distributions returned by this function
    # return the projected distrubution
    return Gaussian(2, mu_pix, cov_pix, self.optical_frame)
