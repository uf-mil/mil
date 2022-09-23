import time

import rich
import rich_click as click
from rich.console import Console


@click.group()
def mission():
    """
    Manage NaviGator missions commands
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
