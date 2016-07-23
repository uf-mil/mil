#!/usr/bin/env python
from __future__ import division

from sub8 import tx_sub
import rospkg
import txros
from sub8_gazebo_tools import BagManager
from diagnostics import gazebo_tests
import argparse
import traceback
import datetime
import time

from collections import deque
from twisted.internet import reactor
import time
import os
import sys

rospack = rospkg.RosPack()
DIAG_OUT_DIR = os.path.join(rospack.get_path('sub8_gazebo'), 'diagnostics/')

'''
The plan is to dynamically bag data by keeping a cache and if something goes wrong keep
the bag. Mission files should be able to catch errors and report them back to this script for logging.

FUTURE:
- Easy tools for plotting odom post-hoc (use sub8_montecarlo)
- Stopping/starting nodes with different params on each run would be "the bee's knees"
    - (Really, any node we are monte-carloing should have dynamic reconfig instead of startup parameters)
'''


class JobManager(object):
    def __init__(self, nh, sub, bag=True, verbose=False):
        self.nh = nh
        self.sub = sub
        self.verbose = verbose
        self.timedout = False
        self.job_queue = deque()
        self.record_bags = bag
        if bag:
            self.bagger = BagManager(self.nh, DIAG_OUT_DIR)
        self.successes = 0
        self.fails = 0
        # TODO: append-to-queue service

    @txros.util.cancellableInlineCallbacks
    def run(self):
        """Let the JobManager free-run"""
        while(len(self.job_queue) > 0):
            yield self.run_next_job()

    @txros.util.cancellableInlineCallbacks
    def run_next_job(self):
        """Run the next job in the job queue"""

        job_constructor, loop_count = self.job_queue.popleft()

        current_job = job_constructor(self.nh)
        yield current_job.initial_setup()

        current_loop = 0

        if self.record_bags:
            self.bagger.start_caching()

        while current_loop < loop_count:
            current_loop += 1
            try:
                yield current_job.setup()
            except Exception, error:
                response = "On test #{}, a {} raised an error on test setup, error:\n{}".format(
                    current_loop,
                    current_job._job_name,
                    error.message
                )
                self.log(response)
                continue

            start_time = self.nh.get_time()
            self.nh.sleep(0.2)

            # Run the job
            try:
                # Return True or False
                success, description = yield current_job.run(self.sub)
            except Exception, error:
                response = "On test #{}, a {} raised an error on run, error:\n{}".format(
                    current_loop,
                    current_job._job_name,
                    error.message
                )
                self.log(response)
                continue

            if success:
                print "JOB - Success"
                self.successes += 1
            else:
                print "JOB - Fail"
                print
                self.fails += 1

                if self.record_bags:
                    yield self.bagger.dump()

            print "JOB - Writing report to file. Do not exit."
            elapsed = self.nh.get_time() - start_time

            actual_time = datetime.datetime.now().strftime('%I:%M:%S.%f')
            success_str = "succeeded" if success else "failed"
            report = (
                "Test #{}/{}: {} at {} (Duration: {}).\n".format(current_loop, loop_count, success_str, actual_time, elapsed) +
                "Test reported: {}\n".format(description) +
                "{0} Successes, {1} Fails, {2} Total \n".format(self.successes, self.fails, current_loop)
            )
            self.log(report)

            print "JOB - Done!"

            print "------------------------------------------------------------------"
            print "{0} Successes, {1} Fails, {2} Total".format(self.successes, self.fails, current_loop)
            print "------------------------------------------------------------------"
            print

            if current_loop > 5 and self.fails / current_loop > .9:
                print "Too many errors. Stopping as to not fill HDD with bags."
                break

        print
        print "JOB - Test Finished!"
        print "JOB - Writing report to file. Do not exit."
        self.log("Time of completion: {0}. \n".format(int(time.time())))

    def log(self, text):
        print "JOB - Logging {}".format(text)
        # a+ creates the file if it doesn't exist
        with open(DIAG_OUT_DIR + 'log.txt', 'a+') as f:
            f.write(text)
            f.write("\n-----------------------------------------------------------\n")

        if self.verbose:
            print text
            print "\n-----------------------------------------------------------\n"

    def queue_job(self, name, runs):
        available_tests = [test_name for test_name in dir(gazebo_tests) if not test_name.startswith('_')]
        assert name in available_tests, "Unknown test, {}".format(name)
        assert isinstance(runs, int), "Cannot do non-integer runs, wtf are you trying to do?"

        job_module = getattr(gazebo_tests, name)
        # This is not a very clean API
        job = getattr(job_module, 'Job')
        self.job_queue.append((job, runs))


@txros.util.cancellableInlineCallbacks
def main(args):
        nh = yield txros.NodeHandle.from_argv('job_runner_controller')

        print 'JOB - getting sub'
        sub = yield tx_sub.get_sub(nh)
        yield sub.last_pose()

        print "JOB - Queueing Jobs...."
        job_manager = JobManager(nh, sub, args.bag, args.verbose)

        try:
            for test_name in args.test_names:
                job_manager.queue_job(test_name, args.iterations)

            print "JOB - Running jobs..."
            yield job_manager.run()

        except Exception:
            traceback.print_exc()

        finally:
            reactor.stop()


if __name__ == '__main__':
    usage_msg = ("Input the name of the test you would like to run.\nExamples: " +
                 "\n\n\trosrun sub8_gazebo job_runner align_marker_test")
    desc_msg = "-- Mr. Job Manager --"
    parser = argparse.ArgumentParser(usage=usage_msg, description=desc_msg)
    parser.add_argument(
        dest='test_names', nargs='+',
        help="The names of the tests you'd like to run (ex: test_align)"
    )
    parser.add_argument(
        '--iterations', type=int, default=2,
        help="The number of iterations (For now, it's universal)"
    )
    parser.add_argument(
        '--bag', action='store_true',
        help="Log the data"
    )
    parser.add_argument(
        '--verbose', '-v', action='store_true',
        help="act verbosely"
    )

    args = parser.parse_args(sys.argv[1:])

    reactor.callWhenRunning(main, args)
    reactor.run()
