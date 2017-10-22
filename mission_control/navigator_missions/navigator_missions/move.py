#!/usr/bin/env python
from txros import util, NodeHandle
from twisted.internet import defer, reactor
from navigator import Navigator
import numpy as np
from mil_tools import numpy_to_point, rosmsg_to_numpy
from geometry_msgs.msg import PoseStamped, PointStamped
from mil_misc_tools import ThrowingArgumentParser, ArgumentParserException


class Move(Navigator):
    def decode_parameters(self, parameters):
        return parameters  # Dont bother trying to see params as json

    @classmethod
    def init(cls):
        parser = ThrowingArgumentParser(description='Command Line Mission Runner',
            usage='Pass any pose editor command with an argument. \n\t forward 1 (moves forward 1 meter) \n\t backward 2ft (moves backward 2 feet)')
        parser.add_argument('command', type=str,
            help='Pose editor command to run')
        parser.add_argument('argument', type=str, default=0,
            help='An argument to pass to the command (distance or angle usally). Optionally a unit can be added if a non-standard unit is desired.')
        parser.add_argument('-m', '--movetype', type=str, default='drive',
            help='Move type. See lqrrt documentation for info on how to use this.')
        parser.add_argument('-f', '--focus', type=str,
            help='Point to focus on. See lqrrt documentation for info on how to use this. If not specified, default to focusing around clicked point. ex. "[10, 2.3, 1]"')
        parser.add_argument('-s', '--sim', action='store_true',
            help='This will run navigator in sim mode (ie. not wait for odom or enu bounds). Don\'t do this on the boat.')
        parser.add_argument('-p', '--plantime', type=float,
            help='Time given to the planner for it\'s first plan.')
        parser.add_argument('-b', '--blind', action='store_true',
            help='Move without looking at the ogrid. DANGEROUS.')
        parser.add_argument('-sf', '--speedfactor', type=str, default="[1, 1, 1]",
            help='Speed to execute the command, don\'t go too much higher than 1 on the real boat. Use like "[1, 1, .2]" to reduce rotation speed to 20% max or just ".5" to reduce x, y, and rotational speed to 50% max.')
        cls.parser = parser

    @util.cancellableInlineCallbacks
    def run(self, parameters):
        if type(parameters) == unicode:
            parameters = str(parameters)
        if type(parameters) != str:
            defer.returnValue('parameters not string')
        argv = parameters.split()
        args = self.parser.parse_args(argv)

        if not self.pose:
            raise Exception('Cant move: No odom')

        action_kwargs = {'move_type': args.movetype}#, 'speed_factor': args.speedfactor}

        action_kwargs['blind'] = args.blind
        yield self.change_wrench('autonomous')
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

        if args.command == 'custom':
            # Let the user input custom commands, the eval may be dangerous so do away with that at some point.
            self.send_feedback("Moving with the command: {}".format(args.argument))
            res = yield eval("self.move.{}.go(move_type='{move_type}')".format(args.argument, **action_kwargs))

        elif args.command == 'rviz':
            self.send_feedback("Moving to last published rviz position")
            sub = nh.subscribe("/rviz_goal", PoseStamped)
            target_pose = yield util.wrap_time_notice(sub.get_next_message(), 2,  "Rviz goal")
            res = yield self.move.to_pose(target_pose).go(**action_kwargs)

        elif args.command == 'circle':
            self.send_feedback("Moving in a circle around last clicked_point")
            sub = nh.subscribe("/rviz_point", PointStamped)
            target_point = yield util.wrap_time_notice(sub.get_next_message(), 2, "Rviz point")
            target_point = rosmsg_to_numpy(target_point.point)
            circle = self.move.circle_point(target_point, float(args.argument))
            direction = 'cw' if args.argument == '-1' else 'ccw'
            res = yield self.move.circle_point(target_point, direction).go()

        else:
            shorthand = {"f":"forward", "b":"backward", "l":"left", "r":"right", "yl":"yaw_left", "yr":"yaw_right"}
            command = args.command if args.command not in shorthand.keys() else shorthand[args.command]
            movement = getattr(self.move, command)

            trans_move = command[:3] != "yaw"
            unit = "m" if trans_move else "rad"
            amount = args.argument
            # See if there's a non standard unit at the end of the argument
            if not args.argument[-1].isdigit():
                last_digit_index = [i for i, c in enumerate(args.argument) if c.isdigit()][-1] + 1
                amount = float(args.argument[:last_digit_index])
                unit = args.argument[last_digit_index:]

            # Get the kwargs to pass to the action server
            station_hold = amount == '0'
            if station_hold:
                action_kwargs['move_type'] = 'hold'

            msg = "Moving {} ".format(command) if trans_move else "Yawing {} ".format(command[4:])
            self.send_feedback(msg + "{}{}".format(amount, unit))
            res = yield movement(float(amount), unit).go(**action_kwargs)
        if res.failure_reason is not '':
            raise Exception('Move failed. Reason: {}'.format(res.failure_reason))
        defer.returnValue('Move completed successfully!')

