#!/usr/bin/env python
from __future__ import division

import rospkg
import txros
from sub8_gazebo_tools import BagManager
from diagnostics import gazebo_tests
from sub8 import tx_sub
import argparse
import traceback

from collections import deque
from twisted.internet import reactor
import time
import os
import sys

rospack = rospkg.RosPack()
DIAG_OUT_DIR = os.path.join(rospack.get_path('sub8_gazebo'), 'diagnostics/')

'''
In its current implementation, to run a mission you should create a mission script that listens to
'/gazebo/job_runner/mission_start' for a start service. Right now the service doesn't provide anything but could
easily provide something to the mission file if we want to. The mission script should return with the status of that run
(pass or fail) and perhapse a short message that will be logged. When this script gets that signal, it will reset the gazebo
world and run the same mission again. The plan is to dynamically bag data by keeping a cache and if something goes wrong keep
the bag. Mission files should be able to catch errors and report them back to this script for logging.

FUTURE:
 - Job Queue
'''


class JobManager(object):
    def __init__(self, nh):
        self.nh = nh
        self.timedout = False
        self.job_queue = deque()

        self.bagger = BagManager(self.nh, DIAG_OUT_DIR)
        self.successes = 0
        self.fails = 0
        # TODO: append-to-queue service
        # self.run_job = nh.get_service_client('/gazebo/job_runner/mission_start', RunJob)

    @txros.util.cancellableInlineCallbacks
    def run(self):
        self.sub = yield tx_sub.get_sub(self.nh)
        while(len(self.job_queue) > 0):
            yield self.run_next_job()

    @txros.util.cancellableInlineCallbacks
    def run_next_job(self):
        """TODO:
        Job queue
        manage current job
        """

        job_constructor, loop_count = self.job_queue.popleft()

        current_job = job_constructor(self.nh)

        current_loop = 0

        self.bagger.start_caching()

        while current_loop < loop_count:
            current_loop += 1
            try:
                yield current_job.setup()
            except Exception, error:
                print "On test #{}, a {} raised an error on test setup".format(
                    current_loop,
                    current_job._job_name
                )
                print error.message
                continue

            yield self.nh.sleep(2.0)
            start_time = self.nh.get_time()

            # Run the job
            try:
                # Return True or False
                success = yield current_job.run(self.sub)
            except Exception, error:
                print "On test #{}, a {} raised an error on run".format(
                    current_loop,
                    current_job._job_name
                )
                print error.message
                continue

            if success:
                print "Success"
                self.successes += 1
            else:
                print "Fail"
                print
                self.fails += 1
                yield self.bagger.dump()

            print "Writing report to file. Do not exit."
            elapsed = self.nh.get_time() - start_time
            try:
                with open(DIAG_OUT_DIR + 'diag.txt', 'a') as f:
                    f.write("Response: {0} occured at {1} (Duration: {2}).\n".format(success, int(time.time()), elapsed))
                    # f.write(message + '\n')
                    f.write("{0} Successes, {1} Fails, {2} Total \n.".format(self.successes, self.fails, current_loop))
                    f.write("-----------------------------------------------------------")
                    f.write('\n')
            except IOError:
                print "Diagnostics file not found. Creating it now."
                with open(DIAG_OUT_DIR + 'diag.txt', 'w') as f:
                    f.write("Response: {0} occured at {1} (Duration: {2}).\n".format(success, int(time.time()), elapsed))
                    # f.write(message + '\n')
                    f.write("{0} Successes, {1} Fails, {2} Total \n.".format(self.successes, self.fails, current_loop))
                    f.write("-----------------------------------------------------------")
                    f.write('\n')
            print
            print "Done!"

            print "------------------------------------------------------------------"
            print "{0} Successes, {1} Fails, {2} Total".format(self.successes, self.fails, current_loop)
            print "------------------------------------------------------------------"
            print

            if current_loop > 5 and self.fails / current_loop > .9:
                print "Too many errors. Stopping as to not fill HDD with bags."
                break
        print
        print "Job Finished!"
        print "Writing report to file. Do not exit."
        with open(DIAG_OUT_DIR + 'diag.txt', 'a') as f:
            f.write("{0} Successes, {1} Fails, {2} Total \n.".format(self.successes, self.fails, current_loop))
            f.write("Time of completion: {0}. \n".format(int(time.time())))
            f.write("-----------------------------------------------------------")
            f.write('\n')

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

        print "Queueing Jobs...."
        job_manager = JobManager(nh)

        try:
            for test_name in args.test_names:
                job_manager.queue_job(test_name, args.iterations)

            print "Running jobs..."
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

    args = parser.parse_args(sys.argv[1:])

    reactor.callWhenRunning(main, args)
    reactor.run()
