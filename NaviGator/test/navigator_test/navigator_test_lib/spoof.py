import genpy
from txros import NodeHandle


class SpoofPubilsher:
    def __init__(self, topic_name, message_type, responses, times):
        self.topic_name = topic_name
        self.responses = responses
        self.times = times
        self.message_type = message_type
        self.total = len(self.responses)

    async def start(self, nh):
        self.my_pub = await nh.advertise(self.topic_name, self.message_type)
        i = 0
        started = nh.get_time()
        while True:
            if len(self.times) == 0:
                break
            if nh.get_time() - started > genpy.Duration(self.times[i % self.total]):
                i += 1
                started = nh.get_time()
            self.my_pub.publish(self.responses[i % self.total])
            await nh.sleep(0.3)

    async def stop(self):
        await self.my_pub.shutdown()


class SpoofService:
    def __init__(self, service_name, message_type, responses):
        self.service_name = service_name
        self.responses = responses
        self.message_type = message_type
        self.total = len(responses)
        self.count = 0
        self.serv = None

    async def start(self, nh: NodeHandle):
        self.serv = nh.advertise_service(
            self.service_name, self.message_type, self._service_cb
        )
        await self.serv.setup()

    def _service_cb(self, req):
        ans = self.responses[self.count % self.total]
        self.count += 1
        return ans

    async def stop(self):
        await self.serv.shutdown()


class SpoofGenerator:
    def spoof_service(self, service_name, message_type, responses):
        return SpoofService(service_name, message_type, responses)

    def spoof_publisher(self, topic_name, message_type, responses, time):
        return SpoofPubilsher(topic_name, message_type, responses, time)
