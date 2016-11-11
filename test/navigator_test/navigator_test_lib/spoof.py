from txros import util
import genpy


class SpoofPubilsher(object):

    def __init__(self, topic_name, message_type, responses, times):
        self.topic_name = topic_name
        self.responses = responses
        self.times = times
        self.message_type = message_type
        self.total = len(self.responses)

    @util.cancellableInlineCallbacks
    def start(self, nh):
        self.my_pub = yield nh.advertise(self.topic_name, self.message_type)
        i = 0
        started = nh.get_time()
        while True:
            if len(self.times) == 0:
                break
            if nh.get_time() - started > genpy.Duration(self.times[i]):
                i += 1
                started = nh.get_time()
            self.my_pub.publish(self.responses[i])
            yield nh.sleep(.3)

    @util.cancellableInlineCallbacks
    def stop(self):
        yield self.my_pub.shutdown()


class SpoofService(object):

    def __init__(self, service_name, message_type, responses):
        self.service_name = service_name
        self.responses = responses
        self.message_type = message_type
        self.total = len(responses)
        self.count = 0

    @util.cancellableInlineCallbacks
    def start(self, nh):
        yield nh.advertise_service(self.service_name, self.message_type, self.service_cb)

    def service_cb(self, req):
        ans = self.responses[self.count % self.total]
        self.count += 1
        return ans


class SpoofGenerator(object):

    def spoof_service(self, service_name, message_type, responses):
        return SpoofService(service_name, message_type, responses)

    def spoof_publisher(self, topic_name, message_type, responses, time):
        return SpoofPubilsher(topic_name, message_type, responses, time)
