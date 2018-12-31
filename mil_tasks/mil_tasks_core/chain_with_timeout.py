from exceptions import TimeoutException, SubmissionException, ParametersException
from twisted.internet import defer
from twisted.python import failure
from txros import util
import json


def MakeChainWithTimeout(base):
    '''
    Generate a ChainWithTimeout mission with the BaseMission specified.
    Used by individual robotics platforms to reuse this example mission.
    '''
    class ChainWithTimeout(base):
        '''
        Example of a mission which runs an arbitrary number of other missions in a linear order. This
        is the mission used by the rqt plugin under the "Chained Missions" section.
        '''
        @util.cancellableInlineCallbacks
        def run_submission_with_timeout(self, mission, timeout, parameters):
            '''
            Runs a child mission, throwing an exception if more time than specified in timeout
            passes before the mission finishes executing.
            '''
            submission = self.run_submission(mission, parameters)
            if timeout == 0:  # Timeout of zero means no timeout
                result = yield submission
                defer.returnValue(result)
            timeout_df = self.nh.sleep(timeout)
            result, index = yield defer.DeferredList([submission, timeout_df], fireOnOneCallback=True,
                                                     fireOnOneErrback=True)
            if index == 0:
                yield timeout_df.cancel()
                defer.returnValue(result)
            if index == 1:
                yield submission.cancel()
                raise TimeoutException(timeout)

        @classmethod
        def decode_parameters(cls, parameters):
            '''
            Goes through list of missions to chain and fills in missing
            attributes like timeout with defaults. If something is invalid,
            raise an exception.
            '''
            parameters = json.loads(parameters)
            if type(parameters) != dict:
                raise ParametersException('must be a dictionary')
            if 'missions' not in parameters:
                raise ParametersException('must have "missions" list')
            if not isinstance(parameters['missions'], list):
                raise ParametersException('"missions" attribute must be a list')
            for mission in parameters['missions']:
                if 'mission' not in mission:
                    raise Exception('invalid parameters, missing attribute "mission"')
                if not cls.has_mission(mission['mission']):
                    raise Exception('mission "{}" not available'.format(mission['mission']))
                if 'parameters' not in mission:
                    mission['parameters'] = ''
                try:
                    mission['parameters'] = cls.get_mission(mission['mission']).decode_parameters(mission['parameters'])
                except Exception as e:
                    raise ParametersException('Invalid parameters for {}: {}'.format(mission['mission'], str(e)))
                if 'timeout' not in mission:
                    mission['timeout'] = 0
                if 'required' not in mission:
                    mission['required'] = True
            return parameters

        @util.cancellableInlineCallbacks
        def run(self, parameters):
            '''
            Runs a list of child missions specified in the parameters with optional timeouts.
            '''
            for mission in parameters['missions']:  # Run each child mission linearly
                def cb(final):
                    '''
                    Called when a submission finishes. If it succeeded, print the result in feedback.
                    If it failed or timedout, print the failure and stop the whole chain if that
                    mission is required.
                    '''
                    if isinstance(final, failure.Failure):
                        self.send_feedback('{} FAILED: {}'.format(mission['mission'], final.getErrorMessage()))
                        if mission['required']:  # Fail whole chain if a required mission times out or fails
                            raise SubmissionException(mission['mission'], final.getErrorMessage())
                    else:
                        self.send_feedback('{} SUCCEEDED: {}'.format(mission['mission'], final))
                        print 'NO FAIL BRO'
                df = self.run_submission_with_timeout(mission['mission'], mission['timeout'], mission['parameters'])
                df.addBoth(cb)
                yield df
            self.send_feedback('Done with all')
            defer.returnValue('All missions complete or skipped.')
    return ChainWithTimeout
