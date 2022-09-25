from operator import ne

import click
import rich
import rich_click as click
from bag import bag
from mission import mission
from nav import nav
from network import network
from rich.console import Console

from docs import docs


@click.group()
def mil():
    pass


mil.add_command(nav)
mil.add_command(mission)
mil.add_command(network)
mil.add_command(docs)
mil.add_command(bag)

if __name__ == "__main__":
    mil()
