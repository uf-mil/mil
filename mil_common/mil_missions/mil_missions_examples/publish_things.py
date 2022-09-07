import genpy
from std_msgs.msg import String
from txros import NodeHandle

from .base_mission import ExampleBaseMission


class PublishThings(ExampleBaseMission):
    publish_time = 5.0
    publish_frequency = 10

    nh: NodeHandle

    @classmethod
    def init(cls):
        cls.publisher = cls.nh.advertise("/test_pub", String)
        print("Publish things created publisher")

    async def run(self, parameters):
        await self.publisher.setup()
        self.send_feedback(f"Printing {parameters} for {self.publish_time} seconds")
        end_time = (self.nh.get_time()) + genpy.Duration(self.publish_time)
        msg = String(parameters)
        while (self.nh.get_time()) < end_time:
            self.publisher.publish(msg)
            await self.nh.sleep(1.0 / self.publish_frequency)
        self.send_feedback("Done publishing.")
        await self.nh.sleep(1.0)
        return "I did it mom!"
