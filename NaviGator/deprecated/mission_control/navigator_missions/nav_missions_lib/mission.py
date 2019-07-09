from txros import util, NodeHandle
from twisted.internet import defer
from mil_misc_tools.text_effects import fprint


class Mission(object):
    """The class that represents a mission."""

    def __init__(self, name, marker, min_time, weight, points, looking_for, mission_deps_left, mission_script=None):
        """Initialize a Mission object."""
        self.name = name
        self.marker = marker
        self.children = []
        self.min_time = min_time
        self.weight = weight
        self.points = points
        self.attempts = 0
        self.start_time = None
        self.timeout = None
        self.looking_for = looking_for
        self.mission_script = mission_script
        self.mission_deps_left = mission_deps_left
        if self.mission_script is None:
            self.mission_script = self.name

    def add_child(self, child):
        """Add child to a mission."""
        self.children.append(child)

    @util.cancellableInlineCallbacks
    def do_mission(self, navigator, planner, module, **kwargs):
        """Perform this mission."""
        yield planner.publish("Starting", self)
        to_run = getattr(module, self.mission_script)
        fprint(self.name, msg_color="green", title="STARTING MISSION")
        res = yield to_run.main(navigator, attempts=self.attempts, **kwargs)
        defer.returnValue(res)

    @util.cancellableInlineCallbacks
    def safe_exit(self, navigator, err, planner, module):
        """Run a safe exit of a mission."""
        try:
            to_run = getattr(module, self.mission_script)
            if hasattr(to_run, 'safe_exit'):
                yield to_run.safe_exit(navigator, err)
                yield planner.publish("SafeExiting")
            else:
                fprint("Hmmmm. This isn't good. Your mission failed, and there was no safe exit. "
                       "I hope this mission doesn't have any children.", msg_color="red")
        except Exception as exp:
            print exp
            fprint("Oh man this is pretty bad, your mission's safe exit failed. SHAME!", msg_color="red")
