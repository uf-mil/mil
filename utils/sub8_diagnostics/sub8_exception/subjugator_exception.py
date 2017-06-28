#!/usr/bin/env python


class SubjuGatorException(BaseException):
    '''
    This class is supposed to be a base for deriving exceptions that are tightly coupled
    to the sub's hardware or architecture as opposed to more generic programatic exceptions

    Examples of appropriate uses are actuator exceptions, thruster exceptions, camera
    exceptions, etc.
    '''

    def __init__(self, *args, **kwargs):
        self.args = args
        self.kwargs = kwargs
