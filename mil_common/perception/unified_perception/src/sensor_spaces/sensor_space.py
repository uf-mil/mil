from abc import ABCMeta, abstractmethod
from threading import Thread, Lock
import tf
import rospy
from rospy.numpy_msg import numpy_msg
from threading import Lock


class SensorSpace():
  __metaclass__ = ABCMeta
  def __init__(self, name, topic, msg_type):
    # should be a list of tuples of (msg, mutex)
    self.msg_buffer = []
    self.edited_msg_buffer = []
    self.name = name
    self.topic = topic
    self.msg_type = numpy_msg(msg_type)
    self.listener = tf.TransformListener()
    self.world_frame = rospy.get_param('world_frame_id')
    self.sub = rospy.Subscriber(self.topic, self.msg_type, self.callback)
    self.lock = Lock()

  def clear_edited_msg_buffer(self):
    self.edited_msg_buffer = []

  @abstractmethod
  def project_to_world_frame(msg):
    pass

  @abstractmethod
  def project_from_world_frame(dist):
    pass

  @abstractmethod
  def apply_score_to(self, msg, new_msgs, idx, score, min_score):
    pass

  @abstractmethod
  def evaluate_distributions(self, dists):
    pass

  def apply_distributions(self, min_score, distributions):
    dists = {}
    for name, dist in distributions.items():
      dists[name] = self.project_from_world_frame(dist)
    score = self.evaluate_distributions(dists)
    # call extract_interesting_region in a new thread on each msg in the message list
    threads = []
    self.lock.acquire()
    to_be_edited_msg_buffer = self.msg_buffer
    self.msg_buffer = []
    self.lock.release()
    start = len(self.edited_msg_buffer)
    self.edited_msg_buffer += [None] * len(to_be_edited_msg_buffer)
    for i, msg in enumerate(to_be_edited_msg_buffer):
      threads.append(Thread(target=self.apply_score_to,
                            args=[msg, self.edited_msg_buffer, i+start, score, min_score]))
      threads[-1].start()
    # wait for them to finish
    for i in threads:
      i.join()
    return

  def callback(self, msg):
    self.lock.acquire()
    self.msg_buffer.append(msg)
    self.lock.release()
