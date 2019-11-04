from twisted.internet import defer
from txros import util
from base_mission import ExampleBaseMission


class SuperMission(ExampleBaseMission):
    def __init__(self, **kwargs):
        super(SuperMission, self).__init__(**kwargs)

    @classmethod
    def init(cls):
        print 'SuperMission init'

    @util.cancellableInlineCallbacks
    def run(self, parameters):
        self.send_feedback('Going to run two missions')
        yield self.nh.sleep(0.5)
        self.send_feedback('Running PrintAndWait')
        ret = yield self.run_submission('PrintAndWait')
        self.send_feedback('PrintAndWait returned {}'.format(ret))
        self.send_feedback('Running PublishThings')
        ret = yield self.run_submission('PublishThings', 'hello world')
        self.send_feedback('PublishThings returned {}'.format(ret))
        if not self.parent:  # Only recurse once
            self.send_feedback('Oh boy! Its bout to get recursive in here.')
            yield self.run_submission('SuperMission')
            self.send_feedback('Recursion works! Thx kev')
        yield self.nh.sleep(0.5)
        defer.returnValue('Woo! Supermissions are awesome!')
