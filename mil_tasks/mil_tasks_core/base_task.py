from txros import util
import json

class BaseTask(object):
    '''
    The base for all tasks used in mil_tasks. Lots of this class
    is just documentation for the various functions that real tasks
    can overload.
    '''
    nh = None
    task_runner = None
    def __init__(self, parent=None):
        '''
        Called when a new instance of a task is created. Tasks
        should be sure to call base task __init__
        def __init__(self):
            super(MyTaskClass, self).__init__()
            ...
        '''
        self.parent = parent

    @classmethod
    def name(cls):
        '''
        Override for real tasks to return a string for how the mission
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
        Override for real tasks. Intended to do things like set up subscribers,
        publishers, load constants from params, etc. Guaranteed to be called
        after base class _init is called, so self.nh is available. Called
        once when imported, then never again.
        '''
        pass

    @classmethod
    def _init(cls, task_runner):
        '''
        Called once for the BaseTask class. Use for base class to set up move
        action clients, perception hook-ins, etc. Other BaseTask classes should first
        call this _init to setup the nodehandle:

        def _init(cls, task_runner):
            super(MyRobotBaseTask, cls)._init(cls, task_runner)
            ...
        '''
        cls.task_runner = task_runner
        cls.nh = cls.task_runner.nh

    def send_feedback(self, message):
        if self.parent:
            self.parent.send_feedback_child(message, self)
        else:
            self.task_runner.send_feedback(message)

    def send_feedback_child(self, message, child):
        self.send_feedback('{}: {}'.format(child.name(), message))

    def has_task(self, name):
        '''
        Returns true if the task runner has a task with specified name
        '''
        return self.task_runner.has_task(name)

    def run_subtask(self, name, parameters=''):
        if not self.has_task(name):
            raise Exception('Cannot run_subtask, \'{}\' unrecognized'.format(name))
        task = self.task_runner.tasks[name](parent=self)
        #self.children.append(task)
        return task.run(parameters)

    def decode_parameters(self, parameters):
        '''
        Override in individual tasks to change how the parameters string is decoded before
        being set to the run() function. By default, attempts to decode the string as json
        and just returns the original string if it fails
        '''
        try:
            return json.loads(parameters)
        except ValueError:
            return parameters

    def cleanup(self):
        '''
        Will be called if a task in canceled, preempted,
        or raises an exception. Called after the run defer is canceled,
        but before any other task is run. It should therefore finish
        very quickly, and only do things like set default parameters for future
        missions, turn off perception, etc.
        '''
        pass

    @util.cancellableInlineCallbacks
    def run(self, parameters):
        print('Base task, just waiting')
        yield self.nh.sleep(1)
