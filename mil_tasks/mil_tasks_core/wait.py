from txros import util
from twisted.internet import defer


def MakeWait(base):
    '''
    Create a Wait task with the specified base task. Used by
    robotics platforms to reuse this task with a different base task.
    '''
    class Wait(base):
        '''
        Class to simply sleep for some time (by default 1 second)
        '''
        DEFAULT_SECONDS = 1.0

        @util.cancellableInlineCallbacks
        def run(self, parameters):
            # If parameter is just a number, sleep this time in seconds
            if type(parameters) == int or type(parameters) == float:
                time = float(parameters)
            # If parameters is a json dictionary, extract the time
            elif type(parameters) == dict and 'time' in parameters:
                time = parameters['time']
                if type(parameters) != int and type(parameters) != float:
                    raise Exception('time parameter must by a number')
            else:
                time = self.DEFAULT_SECONDS
            self.send_feedback('waiting for {} seconds'.format(time))
            yield self.nh.sleep(time)
            defer.returnValue('Done waiting.')
    return Wait
