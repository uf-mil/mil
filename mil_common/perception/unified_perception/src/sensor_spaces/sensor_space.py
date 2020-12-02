from abc import ABCMeta, abstractmethod
from threading import Thread, Lock

import tf

class SensorSpace():
  __metaclass__ = ABCMeta
  def __init__(self, topic):
    self.score = None
    # should be a list of tuples of (msg, mutex)
    self.msg_buffer = []
    self.topic = topic
    self.listener = tf.TransformListener()

  def clear_msg_buffer(self):
    self.reset_score()

    self.musg_buffer = []

  @abstractmethod
  def extract_interesting_region(self, (msg, mutex), probability, min_score):
    pass

  @abstractmethod
  def reset_score(self):
    pass

  def extract_interesting_regions(self, min_score):
    # call extract_interesting_region in a new thread on each msg in the message list
    threads = []
    for i in msg_buffer:
      threads.append(Thread(target=self.extract_interesting_region,
                            args=[i, self.score, min_score]))
      threads[-1].start()
    # wait for them to finish
    for i in threads:
      i.join()
    # return the messages
    return self.msg_buffer

  @abstractmethod
  def add_distribution_from(self, distribution):
    pass

  @abstractmethod
  def callback(msg):
    pass
