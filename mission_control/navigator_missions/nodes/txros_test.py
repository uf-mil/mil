#!/usr/bin/env python

from txros import util
from twisted.internet import threads, reactor
import time


def cb(res):
    print "callback"


def errcb(err):
    print "errback"
    time.sleep(5)
    print "errrbbackkk1"


def la():
    print "11"
    time.sleep(10)
    raise Exception()


@util.cancellableInlineCallbacks
def do_something():
    print "madeoit"
    d = threads.deferToThread(la)
    d.addCallbacks(cb, errback=errcb)
    yield d
    print "done"
    reactor.stop()

reactor.callWhenRunning(do_something)
reactor.run()