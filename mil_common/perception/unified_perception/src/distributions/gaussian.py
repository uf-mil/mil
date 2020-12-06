#!/usr/bin/env python
import numpy
import rospy
import tf
import numpy as np


class Gaussian:
  def __init__(self, n, mu, cov, frame_id):
    self.frame = frame_id
    self.n = n
    if mu.shape == (n,):
      self.mu = mu.reshape((n,1))
    elif mu.shape == (n,1):
      self.mu = mu
    else:
      raise Exception('Indicated size of ' + str(n) + ', but mu is of shape ' + str(mu.shape))

    if cov.shape == (n,n):
      self.cov = cov
    else:
      raise Exception('Indicated size of ' + str(n) + ', but cov is of shape ' + str(cov.shape))


  def transform_to_frame(self, frame, time=rospy.Time(0)):
    (trans, quat) = self.listener.lookupTransform(self.frame_id,
                                                  frame,
                                                  time)
    new_mu = self.mu + trans
    R = tf.transforms.quaternion_matrix(quat)
    sig = np.matmul(R, np.matmul(cov, R.T))
    return (Gaussian(len(new_mu), new_mu, new_cov, frame), trans, quat)


  def __mul__(rhs):
    if rhs.frame_id != self.frame_id:
      raise Exception('Cannot multiply gaussians not in the same frame.\nleft hand side is in ' +
                      self.frame_id + ',\n but right hand side is in ' + rhs.frame_id)
    if rhs.n != self.n:
      raise Exception('Cannot multiply gaussians of not in the same dimentionality.\n' +
                      'left hand side is dimentionality of '  + str(self.n) + ',\n'
                      'but right hand side is of dimentionality of ' + str(rhs.n))
    # math reference: https://math.stackexchange.com/questions/157172/product-of-two-multivariate-gaussians-distributions
    cov = (self.cov, rhs.cov)
    mu = (self.mu, rhs.mu)
    new_cov = (cov[0].inv() + cov[1].inv()).inv()
    new_mu = np.matmul(new_cov, (np.matmul(cov[0].inv(), mu[0]) + np.matmul(cov[1].inv(), mu[1])))
    return Gaussian(self.n, new_mu, new_cov, self.frame)


