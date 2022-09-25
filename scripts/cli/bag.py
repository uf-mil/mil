import time

import rich
import rich_click as click
import rospy
from rich.console import Console


@click.group()
def bag():
    """
    Manage building of MIL documentation
    """
    pass


def validate_name(ctx, param, values):
    topic_names = [t[0] for t in rospy.get_published_topics()]
    for value in values:
        if value not in topic_names:
            raise click.BadParameter(f"{value} is not a valid ROS topic.")
    return values


@bag.command()
@click.argument("names", callback=validate_name, required=False, nargs=-1)
@click.option(
    "--all", help="Record all topics", default=False, required=False, is_flag=True
)
def record(names, all):
    """
    Record the output of a list of topics named NAMES.
    """
    click.echo(f"Doing something with {names}... (all: {all})")


@bag.command()
@click.argument("input_bag")
@click.argument("input_topic")
@click.argument("output_bag")
@click.argument("output_topic")
def retopic(input_bag, input_topic, output_bag, output_topic):
    """
    Rename the INPUT_TOPIC of INPUT_BAG to OUTPUT_TOPIC in OUTPUT_BAG. The original
    bag is not overwritten.
    """
    click.echo(f"{input_bag, input_topic, output_bag, output_topic}")
