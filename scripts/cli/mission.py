import time

import rich
import rich_click as click
from rich.console import Console


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
    pass


@mission.command()
def list():
    """ """
    pass
