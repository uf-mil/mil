import txros
from twisted.internet import defer


class FindTheBreakTestPerception(object):

    def __init__(self, nh):
        self.nh = nh

    @txros.util.cancellableInlineCallbacks
    def count_pipes(self):
        yield self.nh.sleep(5)
        defer.returnValue(4)
