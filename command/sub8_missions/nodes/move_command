#!/usr/bin/env python
from txros import util, NodeHandle
from twisted.internet import defer, reactor

import numpy as np
from mil_misc_tools import text_effects
from sub8 import sub_singleton
from geometry_msgs.msg import PoseStamped, PointStamped, Pose, Point, Quaternion, Twist
from gazebo_msgs.msg import ModelState
import missions
from txros import publisher
from ros_alarms import TxAlarmBroadcaster
import rospkg
import yaml
import os


import argparse

UNITS = {'m': 1, 'ft': 0.3048, 'yard': 0.9144, 'rad': 1, 'deg': 0.0174533}
SHORTHAND = {"f": "forward", "b": "backward", "l": "left", "r": "right", "yl": "yaw_left", "yr": "yaw_right",
             "d": "down", "u": "up"}

ros_t = lambda d: util.genpy.Duration(d)
fprint = text_effects.FprintFactory(title="MOVE_COMMAND").fprint


@util.cancellableInlineCallbacks
def main(args):
    commands = args.commands
    arguments = commands[1::2] #Split into commands and arguments, every other index
    commands = commands[0::2]
    for i in xrange(len(commands)):
        command = commands[i]
        argument = arguments[i]
        nh, _ = yield NodeHandle.from_argv_with_remaining("move_command", anonymous=True)
        available_missions = [mission_name for mission_name in dir(missions) if not mission_name.startswith('_')]

        sub = yield sub_singleton._Sub(nh)._init()
        if args.test:
            fprint("Running in test mode", msg_color='yellow')
            sub.set_test_mode()

        fprint("Waiting for odom...")
        yield sub.tx_pose()
        fprint("Odom found!", msg_color='green')
        action_kwargs = {'speed': float(args.speed), 'blind': bool(args.blind)}

        if command == 'custom':
            # Let the user input custom commands, the eval may be dangerous so do away with that at some point.
            fprint("Moving with the command: {}".format(argument))
            res = yield eval("sub.move.{}.go()".format(argument, **action_kwargs))

        elif command in ['tp', 'teleport']:
            try:
                rospack = rospkg.RosPack()
                state_set_pub = yield nh.advertise('gazebo/set_model_state', ModelState)
                config_file = os.path.join(rospack.get_path('sub8_gazebo'), 'config', 'teleport_locs.yaml')
                f = yaml.load(open(config_file, 'r'))
                if len(arguments) > 1:
                    #Command only takes in one string so to prevent this command from flowing over into movement we break before it proceeds.
                    fprint("Error, more than one argument detected." , msg_color='red')
                    break
                else:
                    try:
                        x = float(argument.split(' ')[0])
                        y = float(argument.split(' ')[1])
                        z = float(argument.split(' ')[2])
                        #Assumption is if we make it this far, we have successfully bound the previous three coordinates.
                        #The below would fail if we entered a location name instead of coords but we should have caught by this point.
                        #This is to catch anything over 3 coordinates. If only two were given then we would also error out above.
                        if len(argument.split(' ')) != 3:
                            fprint("Incorrect number of coordinates", msg_color='red')
                            break
                    except IndexError:
                        fprint("Incorrect number of coordinates", msg_color='red')
                        break
                    except ValueError:
                        try:
                            if argument in ('list', 'l'):
                                fprint('Available TP locations:')
                                for k in f:
                                    print '  * ' + k
                                break
                            argz = f[argument]
                            x = float(argz.split(' ')[0])
                            y = float(argz.split(' ')[1])
                            z = float(argz.split(' ')[2])
                        except LookupError as e:
                            #This means we did not find the saved location referenced by the argument.
                            fprint("TP location not found, check input.", msg_color='red')
                            break
                    modelstate = ModelState(
                        model_name='sub8',
                        pose = Pose(position=Point(x,y,z), orientation=Quaternion(0,0,0,0)),
                        twist = Twist(linear=Point(0,0,0),angular=Point(0,0,0)),
                        reference_frame= 'world')
                        #Sometimes you need to sleep in order to get the thing to publish
                        #Apparently there is a latency when you set a publisher and it needs to actually hook into it.
                        #As an additional note, given the way we do trajectory in the sim, we must kill sub to prevent
                        #the trajectory from overriding our teleport and bringing it back to its expected position.
                    ab = TxAlarmBroadcaster(nh,'kill', node_name='kill')
                    yield ab.raise_alarm(problem_description='TELEPORTING: KILLING SUB',
                            severity=5)
                    yield nh.sleep(1)
                    yield state_set_pub.publish(modelstate)
                    yield nh.sleep(1)
                    yield ab.clear_alarm()

            except KeyboardInterrupt as e:
                #Catches a ctrl-c situation and ends the program. Break will just bring the program to its natural conclusion.
                break

        elif command in ['zrp', 'level_off', 'zpr']:
            fprint("Zeroing roll and pitch")
            res = yield sub.move.zero_roll_and_pitch().go(**action_kwargs)

        elif command == "stop":
            fprint("Stopping...")
            if args.zrp:
                res = yield sub.move.forward(0).zero_roll_and_pitch().go(**action_kwargs)
            else:
                res = yield sub.move.forward(0).go(**action_kwargs)

        else:
            # Get the command from shorthand if it's there
            command = SHORTHAND.get(command, command)
            movement = getattr(sub.move, command)

            trans_move = command[:3] != "yaw" and command[:5] != "pitch" and command[:4] != "roll"
            unit = "m" if trans_move else "rad"

            amount = argument
            # See if there's a non standard unit at the end of the argument
            if not argument[-1].isdigit():
                last_digit_index = [i for i, c in enumerate(argument) if c.isdigit()][-1] + 1
                amount = argument[:last_digit_index]
                unit = argument[last_digit_index:]

            extra = "and zeroing pitch and roll" if args.zrp else ""
            fprint("{}ing {}{} {}".format(command, amount, unit, extra))
            bad_unit = UNITS.get(unit,"BAD")
            if bad_unit == "BAD":
                fprint("BAD UNIT")
                break
            goal = movement(float(amount) * UNITS.get(unit, 1))
            if args.zrp:
                res = yield goal.zero_roll_and_pitch().go(**action_kwargs)
            else:
                res = yield goal.go(**action_kwargs)
            fprint("Result: {}".format(res.error))

    defer.returnValue(reactor.stop())

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Command Line Mission Runner',
        usage='Pass any pose editor commands with an arguments. \n\t forward 1 (moves forward 1 meter) \n\t backward 2ft (moves backward 2 feet) \n\t forward 1 backward 2ft (moves forward 1 meter then from there moves backward 2 feet), ')
    parser.add_argument('commands', type=str, nargs='*',
        help='Pose editor commands to run each followed by an argument to pass to the command (distance or angle usally). Optionally a unit can be added if a non-standard unit is desired.')
    parser.add_argument('-t', '--test', action='store_true',
        help="Runs the mission in test mode and not move the sub.")
    parser.add_argument('-s', '--speed', type=float, default=0.2,
        help="How fast to execute the move. (m/s)")
    parser.add_argument('-z', '--zrp', action='store_true',
        help="Make end goal have zero roll and pitch.")
    parser.add_argument('-b', '--blind', action='store_true',
        help="Do not check if waypoint is safe.")
    args = parser.parse_args()

    reactor.callWhenRunning(lambda: main(args))
    reactor.run()
