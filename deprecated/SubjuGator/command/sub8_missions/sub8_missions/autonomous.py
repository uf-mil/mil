import genpy
# Import missions here
import pinger
import axros
from mil_misc_tools import text_effects
from ros_alarms import TxAlarmBroadcaster, TxAlarmListener
from twisted.internet import defer

fprint = text_effects.FprintFactory(title="AUTO_MISSION").fprint
WAIT_SECONDS = 5.0


@axros.util.cancellableInlineCallbacks
def run_mission(sub, mission, timeout):
    # timeout in seconds
    m = mission.run(sub).addErrback(lambda x: None)
    start_time = yield sub.nh.get_time()
    while sub.nh.get_time() - start_time < genpy.Duration(timeout):
        # oof what a hack
        if len(m.callbacks) == 0:
            m.cancel()
            defer.returnValue(True)
        yield sub.nh.sleep(0.5)
    fprint('MISSION TIMEOUT', msg_color='red')
    m.cancel()
    defer.returnValue(False)


@axros.util.cancellableInlineCallbacks
def do_mission(sub):
    fprint("RUNNING MISSION", msg_color="blue")

    # Chain 1 missions
    try:
        completed = yield run_mission(sub, pinger, 400)
        if not completed:  # if we timeout
            pass
        else:
            if (yield sub.nh.has_param('pinger_where')):
                if (yield sub.nh.get_param('pinger_where')) == 0:
                    fprint('Running roulette')
                    # do roulette
                if (yield sub.nh.get_param('pinger_where')) == 1:
                    fprint('Running cash in')
                    # do cash in challenge

    except Exception as e:
        fprint("Error in Chain 1 missions!", msg_color="red")
        print e

    # Create a mission kill alarm and kill in the final area
    ab = yield TxAlarmBroadcaster.init(sub.nh, "mission-kill")
    yield ab.raise_alarm()
    fprint("MISSION COMPLETE", msg_color="green")


@axros.util.cancellableInlineCallbacks
def _check_for_run(sub, nh, alarm):
    ''' Waits for the network loss alarm to trigger before '''
    if (yield nh.has_param("autonomous")) and (yield nh.get_param("autonomous")):
        fprint("Waiting {} seconds before running missions...".format(WAIT_SECONDS))
        yield nh.sleep(WAIT_SECONDS)
        fprint('Running Missions')
        yield do_mission(sub)
    else:
        fprint("Network loss deteceted but NOT starting mission.", msg_color='red')


@axros.util.cancellableInlineCallbacks
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


@axros.util.cancellableInlineCallbacks
def run(sub):
    al = yield TxAlarmListener.init(sub.nh, "network-loss")
    _auto_param_watchdog(sub.nh)

    call_with_sub = lambda *args: _check_for_run(sub, *args)
    al.add_callback(call_with_sub, call_when_cleared=False)

    fprint("Waiting for network-loss...", msg_color="blue")
    yield defer.Deferred()
