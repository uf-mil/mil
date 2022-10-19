from __future__ import division, print_function

import time

import rospy

import rich
import rich_click as click
from rich.console import Console

from std_srvs.srv import Trigger, TriggerRequest, TriggerResponse
from txros import NodeHandle, Service, action, rosxmlrpc, util

import subprocess
import rosgraph
import rosgraph.masterapi

import os
import socket
import sys



#---------------


@click.group()
def mission():
    """
    Run and manage available missions for a robotic system
    """
    pass


@mission.command()
def run():
    """ """
    pass


@mission.command()
def refresh():
    """ """
    # run mission_client, to execute it's refresh command
    # first, check if the ROS master is running- nothing will work without it
    
    isOnline = False
    
    if rosgraph.is_master_online(): # Checks the master uri and results boolean (True or False)
        print('ROS MASTER is Online')
        isOnline = True
    else:
        print('ROS MASTER is Offline')
        
    if isOnline == True:
        subprocess.call("rosrun mil_missions mission_client refresh", shell=True)
    
    

@mission.command()
def list():
    """ """
    pass
    
