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
    time.sleep(2)


@util.cancellableInlineCallbacks
def do_something():
    print "madeoit"
    poo = "1"

    def ha(err):
        poo = "2"
        print "poo"

    d = threads.deferToThread(la)
    d.addErrback(errcb)
    util.wrap_timeout(d, 1)
    print poo
    yield d
    print "done"
    reactor.stop()

reactor.callWhenRunning(do_something)
reactor.run()
