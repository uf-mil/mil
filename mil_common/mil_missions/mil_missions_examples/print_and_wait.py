from twisted.internet import defer
from txros import util

from .base_mission import ExampleBaseMission


class PrintAndWait(ExampleBaseMission):
    @classmethod
    def init(cls):
        print("PrintAndWait init")

    @util.cancellableInlineCallbacks
    def run(self, parameters):
        self.send_feedback("I like eggs")
        yield self.nh.sleep(1.0)
        self.send_feedback("I like milk")
        yield self.nh.sleep(1.0)
        self.send_feedback("I fear death.")
        defer.returnValue("The darkness isn't so scary")
