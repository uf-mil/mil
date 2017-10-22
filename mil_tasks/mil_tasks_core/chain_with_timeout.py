from twisted.internet import defer
from txros import util


def MakeChainWithTimeout(base):
    '''
    Generate a ChainWithTimeout task with the BaseTask specified.
    Used by individual robotics platforms to reuse this example task.
    '''
    class ChainWithTimeout(base):
        '''
        Example of a task which runs an arbitrary number of other tasks in a linear order. This
        is the task used by the rqt plugin under the "Chained Missions" section.
        '''
        @util.cancellableInlineCallbacks
        def run_subtask_with_timeout(self, task, timeout, parameters):
            '''
            Runs a child task, throwing an exception if more time than specified in timeout
            passes before the task finishes executing.
            '''
            subtask = self.run_subtask(task)
            if timeout == 0:  # Timeout of zero means no timeout
                result = yield subtask
                defer.returnValue(result)
            timeout_df = self.nh.sleep(timeout)
            result, index = yield defer.DeferredList([subtask, timeout_df], fireOnOneCallback=True,
                                                     fireOnOneErrback=True)
            if index == 0:
                yield timeout_df.cancel()
                defer.returnValue(result)
            if index == 1:
                yield subtask.cancel()
                raise Exception('{} timedout'.format(task))

        def verify_parameters(self, parameters):
            '''
            Verifies the parameters are valid for chaining tasks, raising
            an exception otherwise
            '''
            if type(parameters) != list:
                raise Exception('parameters must be a list')
            for mission in parameters:
                if 'task' not in mission:
                    raise Exception('invalid parameters, element missing task')
                if not self.has_task(mission['task']):
                    raise Exception('task "{}" not available'.format(mission['task']))

        @util.cancellableInlineCallbacks
        def run(self, parameters):
            '''
            Runs a list of child tasks specified in the parameters with optional timeouts.

            Currently, if any task timesout or raises an exception, the whole chain is broken and
            is aborted.

            TODO: allow child tasks to be "optional" and not abort the whole chain if they fail or timeout
            '''
            self.verify_parameters(parameters)
            for mission in parameters:  # Run each child task linearly
                task = mission['task']
                timeout = mission['timeout'] if 'timeout' in mission else 0
                parameters = mission['parameters'] if 'parameters' in mission else ''
                self.send_feedback('Running {}'.format(task))
                yield self.run_subtask_with_timeout(task, timeout, parameters)
                self.send_feedback('Done with {}'.format(task))
                yield self.nh.sleep(1.0)
            self.send_feedback('Done with all')
            defer.returnValue('All tasks succeeded!')
    return ChainWithTimeout
