import json
from twisted.internet import defer


class BaseMission(object):
    '''
    The base for all missions used in mil_missions. Lots of this class
    is just documentation for the various functions that real missions
    can overload. Individual ROS robotics platforms should extend this
    base class to provide interfaces to the particular systems on the robot.
    '''
    nh = None
    mission_runner = None

    def __init__(self, parent=None):
        '''
        Called when a new instance of a mission is created. Missions
        should be sure to call base mission __init__
        def __init__(self):
            super(MyMissionClass, self).__init__()
            ...
        '''
        self.parent = parent

    @classmethod
    def name(cls):
        '''
        Override for real missions to return a string for how the mission
        with be referenced in the GUI/CLI. For example, a mission implemented
        in class MyCoolMission might implement
        @classmethod
        def name(cls):
            return 'My cool mission'
        '''
        return cls.__name__

    @classmethod
    def init(cls):
        '''
        Called for each when the server starts up and after the base mission is
        initialized. Intended for missions to setup subscribers, state variables,
        etc that will be shared between individual instances of a mission. For example,
        a mission which moves to the current position of a vision target might subscribe
        to the perception node's topic in init() so that when the mission is run it already
        has the latest position.
        '''
        pass

    @classmethod
    def _init(cls, mission_runner):
        '''
        Called once for the BaseMission class. Use for base class to set up move
        action clients, perception hook-ins, etc. Other BaseMission classes should first
        call this _init to setup the nodehandle:

        def _init(cls, mission_runner):
            super(MyRobotBaseMission, cls)._init(cls, mission_runner)
            ...
        '''
        cls.mission_runner = mission_runner
        cls.nh = cls.mission_runner.nh

    def send_feedback(self, message):
        '''
        Send a string as feedback to any clients monitoring this mission. If the mission is a child
        mission, it will call the send_feedback_child of its parent, allowing missions to choose how to
        use the feedback from its children.
        '''
        if self.parent:
            self.parent.send_feedback_child(message, self)
        else:
            self.mission_runner.send_feedback(message)

    def send_feedback_child(self, message, child):
        '''
        Called by child missions when sending feedback. By default sends this feedback prefixed
        with the name of the child mission.
        '''
        self.send_feedback('{}: {}'.format(child.name(), message))

    @classmethod
    def has_mission(self, name):
        '''
        Returns true if the mission runner has a mission with specified name
        '''
        return self.mission_runner.has_mission(name)

    @classmethod
    def get_mission(self, name):
        return self.mission_runner.get_mission(name)

    def run_submission(self, name, parameters=''):
        '''
        Runs another mission available to the mission server, returning the defered object for the
        missions executation.
        @param name: the name of the submission to spawn as a string. If this mission is unknown,
                     raise an exception
        @param parameters: parameters to pass to the run function of the submission. Note,
                           this function does not call decode_parameters, so parent
                           missions need to do this or otherwise ensure the parameters are in
                           the format expected by the child.
        '''
        if not self.has_mission(name):
            raise Exception('Cannot run_submission, \'{}\' unrecognized'.format(name))
        mission = self.mission_runner.missions[name](parent=self)
        return defer.maybeDeferred(mission.run, parameters)

    @classmethod
    def decode_parameters(cls, parameters):
        '''
        Process parameters string from new mission goal or submission. Should return the
        processes parameters which will be passed to the run function. By default
        returns the json decoded object in the string or, if this fails, just
        the original string.

        If this function throws an exception (such as a ParametersException), the mission will be aborted.
        '''
        try:
            return json.loads(parameters)
        except ValueError:
            return parameters

    def cleanup(self):
        '''
        Will be called if a mission in canceled, preempted,
        or raises an exception. Called after the run defer is canceled,
        but before any other mission is run. It should therefore finish
        very quickly, and only do things like set default parameters for future
        missions, turn off perception, etc.
        '''
        pass

    def run(self, parameters):
        '''
        The actual body of the mission. Should attempt to execute whatever is expected
        of the mission, using the interfaces set up in init() or the base mission to
        command actuator movements, read perception output, etc. Should use self.send_feedback
        to update clients about what the mission is doing at the moment. If something goes wrong,
        raise an exception describing what went wrong and the mission will be aborted and cleanup is called.
        If it executes succesfully, return with defer.returnValue(message) to send a final
        result to the connected clients. Missions can also spawn other missions in the run function
        using run_submission.

        @param parameters: arguments to modify the behavior of the mission. By defaut will be a json decoded
                           object from the string passed in the goal, but can be changed
                           by overridding decode_parameters.
        '''
        pass
