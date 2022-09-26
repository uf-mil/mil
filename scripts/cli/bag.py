import re
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


def validate_timeout(ctx, param, value):
    if value is None:
        return None
    try:
        min, sec = value.split(":")
        min, sec = int(min), int(sec)
        if min < 0 or min > 60 or sec < 0 or sec > 60:  # Prevent 90:82 as "duration"
            raise ValueError()
    except:  # Unable to parse
        raise click.BadParameter(
            f"{value} is not a valid timeout. Please format as mm:ss, such as 01:30."
        )


def validate_filesize(ctx, param, value):
    if value is None:
        return None
    try:
        matches = re.findall(
            r"^(\d+\.?\d*)\s*([MG]B)$", value, re.MULTILINE | re.IGNORECASE
        )
        num, expression = float(matches[0][0]), matches[0][1]
        if expression == "MB":
            return num * 1000000
        elif expression == "GB":
            return num * 1000000000
    except:  # Unable to parse
        raise click.BadParameter(
            f"{value} is not a valid filesize. Please format as xx{{MB|GB}}; for example, 50MB or 4.1GB."
        )


@bag.command()
@click.argument("names", callback=validate_name, required=False, nargs=-1)
@click.option(
    "--all", help="Record all topics", default=False, required=False, is_flag=True
)
@click.option(
    "--timeout",
    callback=validate_timeout,
    required=False,
    help="A duration to stop recording after. Format as mm:ss.",
)
@click.option(
    "--filesize",
    callback=validate_filesize,
    required=False,
    help="A limit on the recorded bag. Format as xx{MB,GB}; for example, 50MB or 4.1GB.",
)
def record(names, all, timeout, filesize):
    """
    Record the output of a list of topics named NAMES.
    """
    click.echo(
        f"Doing something with {names}... (all: {all}, timeout: {timeout}, filesize: {filesize})"
    )


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
