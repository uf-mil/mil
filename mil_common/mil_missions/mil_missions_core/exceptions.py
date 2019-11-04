class MissionException(Exception):
    def __init__(self, message, parameters={}):
        self.message = message
        self.parameters = parameters
        super(Exception, self).__init__(message)


class TimeoutException(Exception):
    '''
    Represents an exception from a mission or submission not finishing within
    the requested time.
    '''
    def __init__(self, timeout):
        '''
        @param timeout: time in seconds mission/submission should have finished in
        '''
        self.timeout = timeout

    def __str__(self):
        return 'failed to finish within {} seconds'.format(self.timeout)


class ParametersException(Exception):
    def __init__(self, msg):
        self.msg = msg

    def __str__(self):
        return 'invalid parameters: {}'.format(self.msg)


class SubmissionException(Exception):
    '''
    Represents an exception encountered while running a submission.
    Keeps the name of the submission which failed, so the user knowns where
    the failure occurred.
    '''
    def __init__(self, mission, exception):
        '''
        @param mission: string name of the mission
        @param exception: original exception raised by the submission
        '''
        self.mission = mission
        self.exception = exception

    def __str__(self):
        return 'In submission {}: {}'.format(self.mission, self.exception)
