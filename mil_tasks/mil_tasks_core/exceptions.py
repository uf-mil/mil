class TaskException(Exception):
    def __init__(self, message, parameters={}):
        self.message = message
        self.parameters = parameters
        super(Exception, self).__init__(message)


class TimeoutException(Exception):
    '''
    Represents an exception from a task or subtask not finishing within
    the requested time.
    '''
    def __init__(self, timeout):
        '''
        @param timeout: time in seconds task/subtask should have finished in
        '''
        self.timeout = timeout

    def __str__(self):
        return 'failed to finish within {} seconds'.format(self.timeout)


class ParametersException(Exception):
    def __init__(self, msg):
        self.msg = msg

    def __str__(self):
        return 'invalid parameters: {}'.format(self.msg)


class SubtaskException(Exception):
    '''
    Represents an exception encountered while running a subtask.
    Keeps the name of the subtask which failed, so the user knowns where
    the failure occurred.
    '''
    def __init__(self, task, exception):
        '''
        @param task: string name of the task
        @param exception: original exception raised by the subtask
        '''
        self.task = task
        self.exception = exception

    def __str__(self):
        return 'In subtask {}: {}'.format(self.task, self.exception)
