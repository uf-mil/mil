import genpy
from std_msgs.msg import String
from twisted.internet import defer
from txros import util

from .base_mission import ExampleBaseMission


class PublishThings(ExampleBaseMission):
    publish_time = 5.0
    publish_frequency = 10

    @classmethod
    def init(cls):
        cls.publisher = cls.nh.advertise("/test_pub", String)
        print("Publish things created publisher")

    @util.cancellableInlineCallbacks
    def run(self, parameters):
        self.send_feedback(f"Printing {parameters} for {self.publish_time} seconds")
        end_time = (yield self.nh.get_time()) + genpy.Duration(self.publish_time)
        msg = String(parameters)
        while (yield self.nh.get_time()) < end_time:
            self.publisher.publish(msg)
            yield self.nh.sleep(1.0 / self.publish_frequency)
        self.send_feedback("Done publishing.")
        yield self.nh.sleep(1.0)
        defer.returnValue("I did it mom!")
