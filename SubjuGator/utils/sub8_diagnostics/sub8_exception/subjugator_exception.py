#!/usr/bin/env python


class SubjuGatorException(BaseException):
    '''
    This class is supposed to be a base for deriving exceptions that are tightly coupled
    to the sub's hardware or architecture as opposed to more generic programatic exceptions

    Examples of appropriate uses are actuator exceptions, thruster exceptions, camera
    exceptions, etc.
    '''

    def __init__(self, *args, **kwargs):
        '''
        The first ordered argument will be considered the Exception description and
        will be the first thing printed as part of the object representation.
        Exception parameters can be included as keyword arguments.
        '''
        self.args = args
        self.kwargs = kwargs

    def __repr__(self):
        if len(self.args) == 0:
            desc = 'No description given'
        else:
            desc = self.args[0]

        return desc + '. Parameters: ' + self.kwargs.__repr__()

    __str__ = __repr__
