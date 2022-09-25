import time

import rich
import rich_click as click
from rich.console import Console


@click.group()
def network():
    """
    Manage the local MIL network
    """
    pass
