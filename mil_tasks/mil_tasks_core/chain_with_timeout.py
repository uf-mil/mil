from exceptions import TimeoutException, SubtaskException, ParametersException
from twisted.internet import defer
from twisted.python import failure
from txros import util
import json


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
            subtask = self.run_subtask(task, parameters)
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
                raise TimeoutException(timeout)

        @classmethod
        def decode_parameters(cls, parameters):
            '''
            Goes through list of tasks to chain and fills in missing
            attributes like timeout with defaults. If something is invalid,
            raise an exception.
            '''
            parameters = json.loads(parameters)
            if type(parameters) != dict:
                raise ParametersException('must be a dictionary')
            if 'tasks' not in parameters:
                raise ParametersException('must have "tasks" list')
            if not isinstance(parameters['tasks'], list):
                raise ParametersException('"tasks" attribute must be a list')
            for task in parameters['tasks']:
                if 'task' not in task:
                    raise Exception('invalid parameters, missing attribute "task"')
                if not cls.has_task(task['task']):
                    raise Exception('task "{}" not available'.format(task['task']))
                if 'parameters' not in task:
                    task['parameters'] = ''
                try:
                    task['parameters'] = cls.get_task(task['task']).decode_parameters(task['parameters'])
                except Exception as e:
                    raise ParametersException('Invalid parameters for {}: {}'.format(task['task'], str(e)))
                if 'timeout' not in task:
                    task['timeout'] = 0
                if 'required' not in task:
                    task['required'] = True
            return parameters

        @util.cancellableInlineCallbacks
        def run(self, parameters):
            '''
            Runs a list of child tasks specified in the parameters with optional timeouts.
            '''
            for task in parameters['tasks']:  # Run each child task linearly
                def cb(final):
                    '''
                    Called when a subtask finishes. If it succeeded, print the result in feedback.
                    If it failed or timedout, print the failure and stop the whole chain if that
                    task is required.
                    '''
                    if isinstance(final, failure.Failure):
                        self.send_feedback('{} FAILED: {}'.format(task['task'], final.getErrorMessage()))
                        if task['required']:  # Fail whole chain if a required task times out or fails
                            raise SubtaskException(task['task'], final.getErrorMessage())
                    else:
                        self.send_feedback('{} SUCCEEDED: {}'.format(task['task'], final))
                        print 'NO FAIL BRO'
                df = self.run_subtask_with_timeout(task['task'], task['timeout'], task['parameters'])
                df.addBoth(cb)
                yield df
            self.send_feedback('Done with all')
            defer.returnValue('All tasks complete or skipped.')
    return ChainWithTimeout
