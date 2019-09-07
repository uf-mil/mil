#!/usr/bin/env python
from .sub_singleton import SubjuGator
from txros import util
from geometry_msgs.msg import Pose, Point, Quaternion, Twist
from gazebo_msgs.msg import ModelState
from ros_alarms import TxAlarmBroadcaster
import rospkg
import yaml
import os
from mil_misc_tools import ThrowingArgumentParser

UNITS = {'m': 1, 'ft': 0.3048, 'yard': 0.9144, 'rad': 1, 'deg': 0.0174533}
SHORTHAND = {
    "f": "strafe_forward", "b": "strafe_backward", "l": "strafe_left", "r": "strafe_right", "rf":
    "forward", "rb": "backward", "rl": "left", "rr": "right", "yl": "yaw_left", "yr": "yaw_right", "pu": "pitch_up", "pd": "pitch_down",
             "d": "down", "u": "up"}


class Move(SubjuGator):

    @classmethod
    def decode_parameters(cls, parameters):
        argv = parameters.split()
        return cls.parser.parse_args(argv)

    @classmethod
    def init(cls):
        parser = ThrowingArgumentParser(
            description='Command Line Mission Runner',
                                        usage='Pass any pose editor commands with an arguments. \n\t\
                                               forward 1 (moves forward 1 meter) \n\t\
                                               backward 2ft (moves backward 2 feet)\
                                               \n\t forward 1 backward 2ft\
                                               (moves forward 1 meter then from there moves backward 2 feet), ')
        parser.add_argument('commands', type=str, nargs='*',
                            help='Pose editor commands to run each followed by an argument to pass to the command\
                                  (distance or angle usally). \
                                  Optionally a unit can be added if a non-standard unit is desired.')
        parser.add_argument('-s', '--speed', type=float, default=0.2,
                            help="How fast to execute the move. (m/s)")
        parser.add_argument('-z', '--zrp', action='store_true',
                            help="Make end goal have zero roll and pitch.")
        parser.add_argument('-b', '--blind', action='store_true',
                            help="Do not check if waypoint is safe.")
        cls.parser = parser

    @util.cancellableInlineCallbacks
    def run(self, args):
        commands = args.commands
        # Split into commands and arguments, every other index
        arguments = commands[1::2]
        commands = commands[0::2]
        for i in xrange(len(commands)):
            command = commands[i]
            argument = arguments[i]
            self.send_feedback("Waiting for odom...")
            yield self.tx_pose()
            self.send_feedback("Odom found!")
            action_kwargs = {'speed': float(
                args.speed), 'blind': bool(args.blind)}

            if command == 'custom':
                # Let the user input custom commands, the eval may be dangerous
                # so do away with that at some point.
                self.send_feedback(
                    "Moving with the command: {}".format(argument))
                res = yield eval("self.move.{}.go()".format(argument, **action_kwargs))

            elif command in ['tp', 'teleport']:
                try:
                    rospack = rospkg.RosPack()
                    state_set_pub = yield self.nh.advertise('gazebo/set_model_state', ModelState)
                    config_file = os.path.join(rospack.get_path(
                        'sub8_gazebo'), 'config', 'teleport_locs.yaml')
                    f = yaml.load(open(config_file, 'r'))
                    if len(arguments) > 1:
                        # Command only takes in one string so to prevent this
                        # command from flowing over into movement we break
                        # before it proceeds.
                        self.send_feedback(
                            "Error, more than one argument detected.")
                        break
                    else:
                        try:
                            x = float(argument.split(' ')[0])
                            y = float(argument.split(' ')[1])
                            z = float(argument.split(' ')[2])
                            # Assumption is if we make it this far, we have successfully
                            # bound the previous three coordinates.
                            # The below would fail if we entered a location name instead of coords
                            # but we should have caught by this point.
                            # This is to catch anything over 3 coordinates. If
                            # only two were given then we would also error out
                            # above.
                            if len(argument.split(' ')) != 3:
                                self.send_feedback(
                                    "Incorrect number of coordinates")
                                break
                        except IndexError:
                            self.send_feedback(
                                "Incorrect number of coordinates")
                            break
                        except ValueError:
                            try:
                                if argument in ('list', 'l'):
                                    self.send_feedback(
                                        'Available TP locations:')
                                    for k in f:
                                        print '  * ' + k
                                    break
                                argz = f[argument]
                                x = float(argz.split(' ')[0])
                                y = float(argz.split(' ')[1])
                                z = float(argz.split(' ')[2])
                            except LookupError:
                                # This means we did not find the saved location
                                # referenced by the argument.
                                self.send_feedback(
                                    "TP location not found, check input.")
                                break
                        modelstate = ModelState(
                            model_name='sub8',
                            pose=Pose(position=Point(x, y, z),
                                      orientation=Quaternion(0, 0, 0, 0)),
                            twist=Twist(linear=Point(0, 0, 0),
                                        angular=Point(0, 0, 0)),
                            reference_frame='world')
                        # Sometimes you need to sleep in order to get the thing to publish
                        # Apparently there is a latency when you set a publisher and it needs to actually hook into it.
                        # As an additional note, given the way we do trajectory in the sim, we must kill sub to prevent
                        # the trajectory from overriding our teleport and
                        # bringing it back to its expected position.
                        ab = TxAlarmBroadcaster(
                            self.nh,
                            'kill',
                            node_name='kill')
                        yield ab.raise_alarm(
                            problem_description='TELEPORTING: KILLING SUB',
                                             severity=5)
                        yield self.nh.sleep(1)
                        yield state_set_pub.publish(modelstate)
                        yield self.nh.sleep(1)
                        yield ab.clear_alarm()

                except KeyboardInterrupt:
                    # Catches a ctrl-c situation and ends the program. Break
                    # will just bring the program to its natural conclusion.
                    break

            elif command in ['zrp', 'level_off', 'zpr']:
                self.send_feedback("Zeroing roll and pitch")
                res = yield self.move.zero_roll_and_pitch().go(**action_kwargs)

            elif command == "stop":
                self.send_feedback("Stopping...")
                if args.zrp:
                    res = yield self.move.forward(0).zero_roll_and_pitch().go(**action_kwargs)
                else:
                    res = yield self.move.forward(0).go(**action_kwargs)

            else:
                # Get the command from shorthand if it's there
                command = SHORTHAND.get(command, command)
                movement = getattr(self.move, command)

                trans_move = command[:3] != "yaw" and command[:
                                                              5] != "pitch" and command[:4] != "roll"
                unit = "m" if trans_move else "rad"

                amount = argument
                # See if there's a non standard unit at the end of the argument
                if not argument[-1].isdigit():
                    last_digit_index = [i for i, c in enumerate(
                        argument) if c.isdigit()][-1] + 1
                    amount = argument[:last_digit_index]
                    unit = argument[last_digit_index:]

                extra = "and zeroing pitch and roll" if args.zrp else ""
                self.send_feedback("{}ing {}{} {}".format(
                    command, amount, unit, extra))
                bad_unit = UNITS.get(unit, "BAD")
                if bad_unit == "BAD":
                    self.send_feedback("BAD UNIT")
                    break
                goal = movement(float(amount) * UNITS.get(unit, 1))
                if args.zrp:
                    res = yield goal.zero_roll_and_pitch().go(**action_kwargs)
                else:
                    res = yield goal.go(**action_kwargs)
                self.send_feedback("Result: {}".format(res.error))
