#!/usr/bin/env python
import argparse

import navigator_tests
from txros import util, NodeHandle
from twisted.internet import defer, reactor
from mil_misc_tools.text_effects import fprint
import inspect
from navigator_test_lib import TestUnit


def _import(module):
    clsmembers = inspect.getmembers(module, inspect.isclass)
    for i in clsmembers:
        if issubclass(i[1], TestUnit):
            return i[1]


@util.cancellableInlineCallbacks
def main():
    """Main method to the test node"""
    nh, args = yield NodeHandle.from_argv_with_remaining("navigator_test")
    available_missions = [mission_name for mission_name in dir(navigator_tests) if mission_name[0] != '_']

    parser = argparse.ArgumentParser(description='NaviGator Test')
    parser.add_argument('tests', type=str, nargs='+',
                        help='Test(s) from the navigator_tests folder to run.')
    args = parser.parse_args(args[1:])

    if "list" in args.tests:
        print "\nAvailable tests:\n   *",
        print '\n   * '.join(available_missions)
        print
        defer.returnValue(reactor.stop())

    for test in args.tests:
        # Make sure all missions exist before we run
        assert test in available_missions, "'{}' test not found".format(test)

    for test in args.tests:
        fprint("Running Test!\n", title="TEST")
        to_run = getattr(navigator_tests, test)
        to_run = _import(to_run)
        to_run = to_run(nh)
        to_run.create_spoofs()
        result = yield to_run.run_tests()

        if result is None:
            fprint("{} finished with no result.".format(test), title="TEST")
        else:
            for r in result:
                fprint("{} finished with:".format(test), title="TEST")
            print r

    defer.returnValue(reactor.stop())


if __name__ == '__main__':
    reactor.callWhenRunning(main)
    reactor.run()

    print "\n"
    fprint("Done!", title="TEST")
