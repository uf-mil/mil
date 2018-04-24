#!/usr/bin/env python
from txros import util
from twisted.internet import defer
from navigator import Navigator
from navigator_path_planner.msg import MoveGoal
import numpy as np
from mil_tools import numpy_to_point, rosmsg_to_numpy
from mil_misc_tools import ThrowingArgumentParser


class Move(Navigator):
    @classmethod
    def decode_parameters(cls, parameters):
        argv = parameters.split()
        return cls.parser.parse_args(argv)

    @classmethod
    def init(cls):
        parser = ThrowingArgumentParser(
            description='Command Line Mission Runner',
            usage='Pass any pose editor command with an argument. \n\t\
                   forward 1 (moves forward 1 meter) \n\t backward 2ft (moves backward 2 feet)')
        parser.add_argument('commands', type=str, nargs='*',
                            help='Pose editor command to run')
        parser.add_argument('-m', '--movetype', type=str, default='drive',
                            help='Move type. See lqrrt documentation for info on how to use this.')
        parser.add_argument('-f', '--focus', type=str,
                            help='Point to focus on. See lqrrt documentation for info on how to use this. \
                                  If not specified, default to focusing around clicked point. ex. "[10, 2.3, 1]"')
        parser.add_argument('-p', '--plantime', type=float,
                            help='Time given to the planner for it\'s first plan.')
        parser.add_argument('-b', '--blind', action='store_true',
                            help='Move without looking at the ogrid. DANGEROUS.')
        parser.add_argument('-sf', '--speedfactor', type=str, default="[1, 1, 1]",
                            help='Speed to execute the command, don\'t go too much higher than 1 on the real boat. \
                                  Use like "[1, 1, .2]" to reduce rotation speed to 20% max or just ".5"\
                                  to reduce x, y, and rotational speed to 50% max.')
        cls.parser = parser

    @util.cancellableInlineCallbacks
    def run(self, args):
        if not self.pose:
            raise Exception('Cant move: No odom')

        commands = args.commands
        arguments = commands[1::2]
        commands = commands[0::2]

        self.send_feedback('Switching trajectory to lqrrt')
        self.change_trajectory('lqrrt')

        self.send_feedback('Switching wrench to autonomous')
        yield self.change_wrench('autonomous')

        for i in xrange(len(commands)):
            command = commands[i]
            argument = arguments[i]

            action_kwargs = {'move_type': args.movetype, 'speed_factor': args.speedfactor}

            action_kwargs['blind'] = args.blind
            if args.speedfactor is not None:
                if ',' in args.speedfactor:
                    sf = np.array(map(float, args.speedfactor[1:-1].split(',')))
                else:
                    sf = [float(args.speedfactor)] * 3

            action_kwargs['speed_factor'] = sf

            if args.plantime is not None:
                action_kwargs['initial_plan_time'] = float(args.plantime)

            if args.focus is not None:
                focus = np.array(map(float, args.focus[1:-1].split(',')))
                focus[2] = 1  # Tell lqrrt we want to look at the point
                point = numpy_to_point(focus)
                action_kwargs['focus'] = point

            if command == 'custom':
                # Let the user input custom commands, the eval may be dangerous so do away with that at some point.
                self.send_feedback("Moving with the command: {}".format(argument))
                res = yield eval("self.move.{}.go(move_type='{move_type}')".format(argument, **action_kwargs))

            elif command == 'rviz':
                self.send_feedback('Select a 2D Nav Goal in RVIZ')
                target_pose = yield util.wrap_time_notice(self.rviz_goal.get_next_message(), 2, "Rviz goal")
                self.send_feedback('RVIZ pose recieved!')
                res = yield self.move.to_pose(target_pose).go(**action_kwargs)

            elif command == 'circle':
                self.send_feedback('Select a Publish Point in RVIZ')
                target_point = yield util.wrap_time_notice(self.rviz_point.get_next_message(), 2, "Rviz point")
                self.send_feedback('RVIZ point recieved!')
                target_point = rosmsg_to_numpy(target_point.point)
                direction = 'cw' if argument == '-1' else 'ccw'
                res = yield self.move.circle_point(target_point, direction=direction).go(**action_kwargs)

            else:
                shorthand = {"f": "forward", "b": "backward", "l": "left",
                             "r": "right", "yl": "yaw_left", "yr": "yaw_right"}
                command = command if command not in shorthand.keys() else shorthand[command]
                movement = getattr(self.move, command)

                trans_move = command[:3] != "yaw"
                unit = "m" if trans_move else "rad"
                amount = argument
                # See if there's a non standard unit at the end of the argument
                if not argument[-1].isdigit():
                    last_digit_index = [i for i, c in enumerate(argument) if c.isdigit()][-1] + 1
                    amount = float(argument[:last_digit_index])
                    unit = argument[last_digit_index:]

                # Get the kwargs to pass to the action server
                station_hold = amount == '0'
                if station_hold:
                    action_kwargs['move_type'] = MoveGoal.HOLD

                msg = "Moving {} ".format(command) if trans_move else "Yawing {} ".format(command[4:])
                self.send_feedback(msg + "{}{}".format(amount, unit))
                res = yield movement(float(amount), unit).go(**action_kwargs)
            if res.failure_reason is not '':
                raise Exception('Move failed. Reason: {}'.format(res.failure_reason))
        defer.returnValue('Move completed successfully!')
