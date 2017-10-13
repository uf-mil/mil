from twisted.internet import defer
from txros import util
from base_task import ExampleBaseTask
from std_msgs.msg import String
import genpy

class PublishThings(ExampleBaseTask):
    publish_time = 5.0
    publish_frequency = 10

    @classmethod
    def init(cls):
        cls.publisher = cls.nh.advertise('/test_pub', String)
        print('Publish things created publisher')

    @util.cancellableInlineCallbacks
    def run(self, parameters):
        self.send_feedback('Printing {} for {} seconds'.format(parameters, self.publish_time))
        end_time = (yield self.nh.get_time()) + genpy.Duration(self.publish_time)
        msg = String(parameters)
        while (yield self.nh.get_time()) < end_time:
            self.publisher.publish(msg)
            yield self.nh.sleep(1.0 / self.publish_frequency)
        self.send_feedback('Done publishing.')
        yield self.nh.sleep(1.0)
        defer.returnValue('I did it mom!')
