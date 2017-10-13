from twisted.internet import defer
from txros import util

def MakeChainWithTimeout(base):
    class ChainWithTimeout(base):
        def __init__(self):
            super(ChainWithTimeout, self).__init__()

        @classmethod
        def init(cls):
            print('Chained missions init')

        @util.cancellableInlineCallbacks
        def run_subtask_with_timeout(self, task, timeout, parameters):
            subtask = self.run_subtask(task)
            if timeout == 0:
                result = yield subtask
                defer.returnValue(result)
            timeout_df = self.nh.sleep(timeout)
            result, index = yield defer.DeferredList([subtask, timeout_df], fireOnOneCallback=True, fireOnOneErrback=True)
            if index == 0:
                yield timeout_df.cancel()
                defer.returnValue(result)
            if index == 1:
                yield subtask.cancel()
                raise Exception('{} timedout'.format(task))

        def verify_parameters(self, parameters):
            if type(parameters) != list:
                raise Exception('parameters must be a list')
            for mission in parameters:
                if 'task' not in mission:
                    raise Exception('invalid parameters, element missing task')
                if not self.has_task(mission['task']):
                    raise Exception('task "{}" not available'.format(task))

        @util.cancellableInlineCallbacks
        def run(self, parameters):
            self.verify_parameters(parameters)
            for mission in parameters:
                task = mission['task']
                timeout = mission['timeout'] if 'timeout' in mission else 0
                parameters = mission['parameters'] if 'parameters' in mission else ''
                self.send_feedback('Running {}'.format(task))
                yield self.run_subtask_with_timeout(task, timeout, parameters)
                self.send_feedback('Done with {}'.format(task))
                yield self.nh.sleep(1.0)
            self.send_feedback('Done with all')
            defer.returnValue('we good m8')
    return ChainWithTimeout
