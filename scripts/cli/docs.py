import time

import rich
import rich_click as click
from rich.console import Console


@click.group()
def docs():
    """
    Manage building of MIL documentation
    """
    pass
