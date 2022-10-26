import os
import socket
import subprocess
import sys
import time

import rich
import rich_click as click
import rosgraph
import rosgraph.masterapi
import rospy
from rich.console import Console
from std_srvs.srv import Trigger, TriggerRequest, TriggerResponse
from txros import NodeHandle, Service, action, rosxmlrpc, util

# ---------------


@click.group()
def mission():
    """
    Run and manage available missions for a robotic system.
    """
    pass


@mission.command()
def run():
    """
    Run a specific mission listed in the mission server.
    """
    pass


@mission.command()
def refresh():
    """
    Refresh the list of available missions.
    """
    # run mission_client, to execute it's refresh command
    # first, check if the ROS master is running- nothing will work without it
    if rosgraph.is_master_online():
        subprocess.call("rosrun mil_missions mission_client refresh", shell=True)
    else:
        rich.print("[red1 bold]Cannot find a running ROS instance...")


@mission.command()
def list():
    """
    List all missions that can be run.
    """
    pass
