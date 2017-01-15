import txros
from twisted.internet import defer
from ros_alarms import TxAlarmListener, TxAlarmBroadcaster
from sub8_tools import text_effects 

# Import missions here
import square

fprint = text_effects.FprintFactory(title="AUTO_MISSION").fprint

@txros.util.cancellableInlineCallbacks
def do_mission(sub):
    fprint("RUNNING MISSION", msg_color="blue")
    
    # Chain 1 missions
    try:
       yield square.run(sub) 
    except Exception as e:
        fprint("Error in Chain 1 missions!", msg_color="red")
        print e
    
    # Create a mission kill alarm and kill in the final area
    ab = yield TxAlarmBroadcaster()._init(sub.nh, "mission-kill")
    yield ab.raise_alarm()
    fprint("MISSION COMPLETE", msg_color="green")

@txros.util.cancellableInlineCallbacks
def check_for_run(sub, nh, alarm):
    if (yield nh.has_param("autonomous")) and (yield nh.get_param("autonomous")):
        yield do_mission(sub)

@txros.util.cancellableInlineCallbacks
def run(sub):
    al = yield TxAlarmListener()._init(sub.nh, "network-loss")
    call_with_sub = lambda *args: check_for_run(sub, *args)
    al.add_callback(call_with_sub, call_when_cleared=False)

    fprint("Waiting for network-loss...", msg_color="blue")
    yield defer.Deferred()
