#!/usr/bin/env python3
"""
Script to run all catkin tests in a specified directory. Useful in CI systems where you only need
to test some of the packages

Example usage:
    rosrun mil_tools catkin_tests_directory.py src/NaviGator
"""

import argparse
import subprocess
import sys


def get_tests(directory):
    cmd = f"make -C {directory}"
    cmd += "  -qp | awk -F':' '/^[a-zA-Z0-9][^$#\\/\t=]*:([^=]|$)/ {split($1,A,/ /);for(i in A)print A[i]}'"
    tests = subprocess.check_output(["bash", "-c", cmd])
    tests = tests.split()
    return tests


def get_packages(directory):
    cmd = f"catkin_topological_order --only-names {directory}"
    packages = subprocess.check_output(["bash", "-c", cmd])
    packages = packages.split()
    return packages


def run_tests(tests):
    cmd = "catkin_make " + " ".join(tests)
    print(cmd)
    return subprocess.call(cmd.split())


if __name__ == "__main__":
    # parse configuration from arguments
    parser = argparse.ArgumentParser(
        description="Prints all catkin test targets in a specified directory"
    )
    parser.add_argument(
        "directory",
        type=str,
        default="src",
        nargs="?",
        help="directory to print all tests from",
    )
    parser.add_argument(
        "-b",
        "--build-dir",
        dest="build_dir",
        type=str,
        default="build",
        help="build directory of workspace with tests",
    )
    parser.add_argument(
        "--ignore-dir",
        "-i",
        dest="ignore",
        type=str,
        nargs="*",
        help="packages to ignore",
    )
    args = parser.parse_args(sys.argv[1:])

    # get all available tests in workspace
    tests = get_tests(args.build_dir)
    if len(tests) < 1:
        print("no tests found")
        exit(1)

    # get all packages in specified directory
    packages = get_packages(args.directory)
    if args.ignore is not None:  # Filter out ignored packages
        packages = [p for p in packages if p not in args.ignore]
    if len(packages) < 1:
        print("no packages found")
        exit(1)

    # produce a list of all test build targets from a package in the specified folders
    valid = []
    for p in packages:
        test = f"run_tests_{p}"
        if test in tests:
            valid.append(test)
    if len(valid) < 1:
        print("No tests found")
        exit(1)

    # run catkin
    exit(run_tests(valid))
