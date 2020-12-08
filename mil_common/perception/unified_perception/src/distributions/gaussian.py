#!/usr/bin/env python
import numpy
import rospy
import tf
import numpy as np
from numpy.linalg import inv
from visualization_msgs.msg import Marker


class Gaussian:
  def __init__(self, n, mu, cov, frame_id):
    self.frame = frame_id
    self.n = n
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


  def transform_to_frame(self, frame, listener, time=rospy.Time(), timeout=rospy.Duration(1.0)):
    start_time = rospy.Time.now()
    while rospy.Time.now() - start_time < timeout:
      try:
        (trans, quat) = listener.lookupTransform(self.frame,
                                                 frame,
                                                 time)
      except tf.ExtrapolationException as e:
        continue
    (trans, quat) = listener.lookupTransform(self.frame,
                                             frame,
                                             time)
    # transform mu into the new frame
    new_mu = np.squeeze((self.mu.T + trans).T)
    new_mu =  np.linalg.norm(new_mu) * tf.transformations.quaternion_multiply(
                np.hstack((tf.transformations.unit_vector(new_mu), np.zeros((1,)))),
                quat)[:3]
    # roatate the convariance to match the new frame
    # math reference: https://robotics.stackexchange.com/questions/2556/how-to-rotate-covariance
    R = tf.transformations.quaternion_matrix(quat)[:3,:3]

    new_cov = np.matmul(R, np.matmul(self.cov, R.T))
    return (Gaussian(len(new_mu), new_mu, new_cov, frame), trans, quat)


  def to_rviz_marker(self):
    (e_vals, e_vecs) = np.linalg.eig(self.cov)

    print e_vecs

    marker = Marker()
    marker.header.frame_id = self.frame
    marker.header.stamp = rospy.Time.now()
    marker.type = Marker.SPHERE
    marker.action = Marker.ADD

    marker.scale.x = e_vals[0]
    marker.scale.y = e_vals[1]
    marker.scale.z = e_vals[2]

    tf.rotation_matrix(e_vecs[:,0], e_vecs[:,1], e_vecs[:,2])


    return 0

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
    new_cov = inv((inv(cov[0]) + inv(cov[1])))
    new_mu = np.matmul(new_cov, (np.matmul(inv(cov[0]), mu[0]) + np.matmul(inv(cov[1]), mu[1])))
    return Gaussian(self.n, new_mu, new_cov, self.frame)
