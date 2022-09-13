#!/usr/bin/env python3
import argparse
import asyncio
import inspect

import navigator_tests
import uvloop
from mil_misc_tools.text_effects import fprint
from navigator_test_lib import TestUnit
from txros import NodeHandle


def _import(module):
    clsmembers = inspect.getmembers(module, inspect.isclass)
    for i in clsmembers:
        if issubclass(i[1], TestUnit):
            return i[1]


async def main():
    """Main method to the test node"""
    nh, args = NodeHandle.from_argv_with_remaining("navigator_test")
    await nh.setup()
    available_missions = [
        mission_name for mission_name in dir(navigator_tests) if mission_name[0] != "_"
    ]

    parser = argparse.ArgumentParser(description="NaviGator Test")
    parser.add_argument(
        "tests",
        type=str,
        nargs="+",
        help="Test(s) from the navigator_tests folder to run.",
    )
    args = parser.parse_args(args[1:])

    if "list" in args.tests:
        print(
            "\nAvailable tests:\n   *",
        )
        print("\n   * ".join(available_missions))
        print()
        return

    for test in args.tests:
        # Make sure all missions exist before we run
        assert test in available_missions, f"'{test}' test not found"

    for test in args.tests:
        fprint("Running Test!\n", title="TEST")
        to_run = getattr(navigator_tests, test)
        to_run = _import(to_run)
        to_run = to_run(nh)
        to_run.create_spoofs()
        result = to_run.run_tests()

        if result is None:
            fprint(f"{test} finished with no result.", title="TEST")
        else:
            for r in result:
                fprint(f"{test} finished with:", title="TEST")
                print(r)

    return


if __name__ == "__main__":
    uvloop.install()
    asyncio.run(main())
    print("\n")
    fprint("Done!", title="TEST")
