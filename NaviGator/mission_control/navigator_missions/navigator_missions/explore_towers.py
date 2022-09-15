#!/usr/bin/env python3
import numpy as np
from mil_misc_tools import ThrowingArgumentParser

from .navigator import Navigator


class ExploreTowers(Navigator):
    @classmethod
    def decode_parameters(cls, parameters):
        argv = parameters.split()
        return cls.parser.parse_args(argv)

    @classmethod
    def init(cls):
        parser = ThrowingArgumentParser(
            description="Explore a bit mission",
            usage="Default parameters: 'runtask ExploreJaxon'",
        )
        parser.add_argument(
            "-c",
            "--count",
            type=int,
            default=5,
            help="number of unclassified objects to attempt",
        )
        parser.add_argument(
            "-d", "--dist", type=float, default=30.0, help="distance to limit checks to"
        )
        parser.add_argument(
            "-b",
            "--backup",
            type=float,
            default=7.0,
            help="distance to limit checks to",
        )
        parser.add_argument(
            "-w", "--wait", type=float, default=8.0, help="distance to limit checks to"
        )
        cls.parser = parser

    async def run(self, args):
        initial_boat_pose = (self.tx_pose)[0]

        closest_unclassified = await self.get_sorted_objects(
            "UNKNOWN", n=args.count, throw=False
        )
        closest_unclassified = closest_unclassified[1]

        self.send_feedback("Enableling totem vision")
        await self.set_vision_obstacle_course()

        for unclass_obj in closest_unclassified:
            if np.linalg.norm(unclass_obj - initial_boat_pose) > args.dist:
                self.send_feedback("Object is too far away, skpping.")
                continue
            self.send_feedback("Exploring object")
            await self.move.look_at(unclass_obj).set_position(unclass_obj).backward(
                args.backup
            ).go()
            await self.nh.sleep(args.wait)

        self.send_feedback("Turning vision off")
        await self.set_vision_off()
        return True
