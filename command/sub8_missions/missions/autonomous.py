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
    ab = yield TxAlarmBroadcaster.init(sub.nh, "mission-kill")
    yield ab.raise_alarm()
    fprint("MISSION COMPLETE", msg_color="green")


@txros.util.cancellableInlineCallbacks
def _check_for_run(sub, nh, alarm):
    ''' Waits for the network loss alarm to trigger before '''
    if (yield nh.has_param("autonomous")) and (yield nh.get_param("autonomous")):
        fprint("Preparing to go...")
        yield nh.sleep(5)
        yield do_mission(sub)
    else:
        fprint("Network loss deteceted but NOT starting mission.", msg_color='red')


@txros.util.cancellableInlineCallbacks
def _auto_param_watchdog(nh):
    ''' Watch the `autonomous` param and notify the user when events happen  '''
    ready = False
    while True:
        yield nh.sleep(0.1)

        if not (yield nh.has_param("autonomous")):
            if ready:
                ready = False
                fprint("Autonomous mission disarmed.")
            continue

        if (yield nh.get_param("autonomous")) and not ready:
            ready = True
            fprint("Autonomous mission armed. Disconnect now to run.", msg_color="yellow")

        elif not (yield nh.get_param("autonomous")) and ready:
            ready = False
            fprint("Autonomous mission disarmed.")


@txros.util.cancellableInlineCallbacks
def run(sub):
    al = yield TxAlarmListener.init(sub.nh, "network-loss")
    _auto_param_watchdog(sub.nh)

    call_with_sub = lambda *args: _check_for_run(sub, *args)
    al.add_callback(call_with_sub, call_when_cleared=False)

    fprint("Waiting for network-loss...", msg_color="blue")
    yield defer.Deferred()
