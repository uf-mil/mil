#!/usr/bin/env python
import numpy
import rospy
from rospy.numpy_msg import numpy_msg
import tf
import tf2_ros
import numpy as np
from numpy.linalg import inv
from geometry_msgs.msg import PoseWithCovarianceStamped
from visualization_msgs.msg import Marker
from mil_msgs.msg import GaussianDistribution, GaussianDistribution2D
from scipy.stats import multivariate_normal



class Gaussian:
  def __init__(self, n, mu, cov, frame_id, classification=''):
    self.frame = frame_id
    self.n = n
    self.classification = classification[:]
    if mu.shape == (n,1):
      self.mu = mu.reshape((n,))
    elif mu.shape == (n,):
      self.mu = mu
    else:
      raise Exception('Indicated size of ' + str(n) + ', but mu is of shape ' + str(mu.shape))

    if cov.shape == (n*n,):
      self.cov = cov.reshape(n,n)
    elif cov.shape == (n,n):
      self.cov = cov
    else:
      raise Exception('Indicated size of ' + str(n) + ', but cov is of shape ' + str(cov.shape))
    self.pdf_gen = multivariate_normal(self.mu, self.cov.T)


  def transform_to_frame(self, frame, listener, time, timeout=rospy.Duration(100)):
    got_transform = False
    start = rospy.Time.now()
    #while not got_transform and rospy.Time.now()-start < timeout:
    #  try:
    listener.waitForTransform(self.frame, frame, time, timeout = timeout)
    (trans, quat) = listener.lookupTransform(self.frame,
                                                 frame,
                                                 time)
    #  except tf2_ros.TransformException as e:
    #    continue
    #  got_transform = True
    #if not got_transform:
    #    (trans, quat) = listener.lookupTransform(self.frame,
    #                                             frame,
    #                                             time)
    # transform mu into the new frame
    # math reference: https://math.stackexchange.com/questions/40164/how-do-you-rotate-a-vector-by-a-unit-quaternion
    new_mu = self.mu - trans
    new_mu_u = np.hstack((tf.transformations.unit_vector(new_mu), np.zeros(1,)))
    quat_p = tf.transformations.quaternion_conjugate(quat)
    new_mu = np.linalg.norm(new_mu) * tf.transformations.quaternion_multiply(
                                      tf.transformations.quaternion_multiply(quat_p, new_mu_u),
                                      quat)[:3]


    # roatate the convariance to match the new frame
    # math reference: https://robotics.stackexchange.com/questions/2556/how-to-rotate-covariance
    R = tf.transformations.quaternion_matrix(quat)[:3,:3]
    new_cov = np.matmul(np.matmul(R.T, self.cov.copy()), R)
    return (Gaussian(len(new_mu), new_mu, new_cov, frame,
                     classification=self.classification), trans, quat)


  def to_rviz_marker(self, color=(0.5, 0.0, 1.0, 0.0)):
    '''makes into Pose with Covariance where the orientation is straight up
    '''
    m = Marker()
    m.header.frame_id = self.frame
    m.header.stamp = rospy.Time.now()
    m.type = Marker.SPHERE
    m.pose.position.x = self.mu[0]
    m.pose.position.y = self.mu[1]
    m.pose.position.z = self.mu[2]

    # build an elliplse to represent the Cov
    # reference: https://geus.wordpress.com/2011/09/15/how-to-represent-a-3d-normal-function-with-ros-rviz/
    (e_vals, e_vecs) = numpy.linalg.eig (self.cov)
    e_x = e_vecs[0, :] / np.linalg.norm(e_vecs[0, :])
    e_y = e_vecs[1, :] / np.linalg.norm(e_vecs[1, :])
    e_z = e_vecs[2, :] / np.linalg.norm(e_vecs[2, :])

    R = np.eye(4)
    R[0:3, 0:3] = np.array([e_x, e_y, e_z])

    quat = tf.transformations.quaternion_from_matrix(R)

    m.pose.orientation.x = quat[0]
    m.pose.orientation.y = quat[1]
    m.pose.orientation.z = quat[2]
    m.pose.orientation.w = quat[3]

    m.scale.x = e_vals[0]
    m.scale.y = e_vals[1]
    m.scale.z = e_vals[2]

    m.color.a = 0.5
    m.color.r = 0.0
    m.color.g = 1.0
    m.color.b = 0.0

    return m


  def __mul__(self, rhs):
    if rhs.frame != self.frame:
      raise Exception('Cannot multiply gaussians not in the same frame.\nleft hand side is in ' +
                      self.frame + ',\n but right hand side is in ' + rhs.frame)
    if rhs.n != self.n:
      raise Exception('Cannot multiply gaussians of not in the same dimentionality.\n' +
                      'left hand side is dimentionality of '  + str(self.n) + ',\n'
                      'but right hand side is of dimentionality of ' + str(rhs.n))
    # math reference: https://math.stackexchange.com/questions/157172/product-of-two-multivariate-gaussians-distributions
    cov = (self.cov, rhs.cov)
    mu = (self.mu, rhs.mu)
    new_cov = np.matmul(np.matmul(cov[0], np.linalg.inv(cov[0] + cov[1])), cov[1])
    new_mu = np.matmul(np.matmul(cov[1], np.linalg.inv(cov[0] + cov[1])), mu[0]) + \
             np.matmul(np.matmul(cov[0], np.linalg.inv(cov[0] + cov[1])), mu[1])

    return Gaussian(self.n, new_mu, new_cov, self.frame, classification=self.classification)


  def to_msg(self):
    if self.n == 3:
      gd = GaussianDistribution()
      gd.mu = list(self.mu)
      gd.cov = list(np.ravel(self.cov))
      gd.id = ''
      gd.classification = self.classification
      return gd
    elif self.n == 2:
      gd = GaussianDistribution2D()
      gd.mu = list(self.mu)
      gd.cov = list(np.ravel(self.cov))
      gd.id = ''
      gd.classification = self.classification
      return gd
    else:
      raise Exception(str(n) + ' dimentional gaussians have no corresponding ros msg specified')


  def pdf(self, *args, **kwargs):
    return self.pdf_gen.pdf(*args, **kwargs)


  def bhattacharyya_distance(self, g):
    # math reference: https://en.wikipedia.org/wiki/Bhattacharyya_distance#Bhattacharyya_coefficient
    if self.frame != g.frame:
      raise Exception('in order to take a meaningful Bhattacharyya distance, ' +
                      'the two gaussians must be the same')
    cov = (self.cov + g.cov)/(2)
    D = (1/8.0)*np.matmul((self.mu - g.mu).T, np.matmul(np.linalg.inv(cov), (self.mu-g.mu))) + \
        (1/2.0)*np.log(np.linalg.det(cov) /
                       np.sqrt(np.linalg.det(self.cov)*np.linalg.det(g.cov)))
    return D


