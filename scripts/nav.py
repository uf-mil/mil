import time

import rich
import rich_click as click
from rich.console import Console


@click.group()
def nav():
    """
    Command-line interface to all tools for NaviGator.
    """
    pass


@nav.command()
def rviz():
    """
    Launch the NaviGator configuration for Rviz.
    """
    click.echo("Initialized the database")


@nav.command()
@click.argument("thruster", type=click.Choice(["FR", "FL", "BR", "BL"]), required=False)
@click.argument("setpoint", default=10, type=click.IntRange(0, 100), required=False)
@click.option(
    "--all",
    "-a",
    help="Spin all thrusters.",
    default=False,
    is_flag=True,
)
def thrust(thruster: str, setpoint: int, all: bool):
    """
    Spin a THRUSTER on NaviGator with SETPOINT amount of thrust.
    """
    if thruster is None and all is False:
        rich.print(
            "[red1]Please choose a [bold]THRUSTER[/bold] (FR, FL, BR, BL) to spin if not using [bold]--all[/bold]."
        )
        return

    def spinning(_thruster: str) -> bool:
        if all:
            return True
        return _thruster == thruster

    count = 0
    c = Console()
    c.print(
        f""" {'[blink chartreuse1]Spinning[/]' if spinning('FR') else '        '}            {'[blink chartreuse1]Spinning[/]' if spinning('BR') else '        '}
   {'[blink chartreuse1 bold]vvv[/]' if spinning('FR') else '   '}                 {'[bold chartreuse1]vvv[/]' if spinning('BR') else '   '}
[grey42]┌──────────────────────────┐
└───┼──┼───────┼────────┼──┘[/]
[grey62]    │  ├───────┤        │
    │  │◄◄◄◄◄◄◄│        │
    │  │◄◄◄◄◄◄◄│        │
    │  ├───────┤        │[/]
[grey42]┌───┼──┼───────┼────────┼──┐
└──────────────────────────┘[/]
   {'[blink chartreuse1 bold]^^^[/]' if spinning('FL') else '   '}                 {'[bold chartreuse1]^^^[/]' if spinning('BL') else '   '}
{'[blink chartreuse1]Spinning[/]' if spinning('FL') else '        '}             {'[blink chartreuse1]Spinning[/]' if spinning('BL') else '   '}
    """
    )
    while True:
        c.print(
            f"Spinning {thruster if not all else 'all thrusters'} with {setpoint} amount of thrust for {count:.1f} seconds. ({count*setpoint*6:.0f} attempted spins)",
            end="\r",
        )
        time.sleep(0.01)
        count += 0.01


@nav.group()
def control():
    """
    Manage control of NaviGator through external controllers.
    """
    pass


@control.command()
def xbox():
    """
    Control NaviGator over Xbox controller.
    """
    rich.print(
        f"""
    List of Controls:

        [grey62]A - Unkill[/]
        [bold chartreuse1]B - Kill[/]
        [grey62]X - Toggle Move[/]
    """
    )


@control.command()
def keyboard():
    """
    Control NaviGator over keyboard.
    """
    pass


@nav.group()
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


if __name__ == "__main__":
    nav()
