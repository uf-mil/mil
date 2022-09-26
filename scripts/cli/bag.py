import functools
import importlib
import os
import re
import time

import rich
import rich_click as click
import rosbag
import rospy
from rich.console import Console
from rich.progress import (
    BarColumn,
    DownloadColumn,
    FileSizeColumn,
    Progress,
    TaskID,
    TextColumn,
    TimeElapsedColumn,
    TimeRemainingColumn,
    TransferSpeedColumn,
)


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
    return [t for t in rospy.get_published_topics() if t[0] in values]


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


def write_to_bag(bag, msg):
    bag.write("test", msg)


# Thanks Fred! https://stackoverflow.com/a/1094933
def sizeof_fmt(num, suffix="B"):
    for unit in ["", "K", "M", "G", "T", "P", "E", "Z"]:
        if abs(num) < 1000.0:
            return f"{num:3.1f}{unit}{suffix}"
        num /= 1000.0
    return f"{num:.1f}Y{suffix}"


@bag.command()
@click.argument("filename", type=str, required=True)
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
def record(filename, names, all, timeout, filesize):
    """
    Record the output of a list of topics named NAMES to a bag named FILENAME.
    """
    click.echo(
        f"Doing something with {names} using {filename}... (all: {all}, timeout: {timeout}, filesize: {filesize})"
    )
    bag = rosbag.Bag(filename, "w")

    progress = Progress(
        TextColumn("[bold blue]{task.fields[filename]}", justify="right"),
        BarColumn(),
        "[progress.percentage]{task.percentage:>3.1f}%",
        "•",
        FileSizeColumn(),
        "•",
        TextColumn("[bold blue]{task.fields[messages]}"),
        "•",
        TransferSpeedColumn(),
        "•",
        TimeElapsedColumn(),
    )
    task_id = progress.add_task("download", filename=filename, messages=0, start=False)

    subs = []
    rospy.init_node("mil_cli_listener", anonymous=True)
    for topic_name, msg_name in names:
        msg_mod, class_name = msg_name.split("/")
        ros_pkg = importlib.import_module(".msg", package=msg_mod)
        subs.append(
            rospy.Subscriber(
                topic_name,
                getattr(ros_pkg, class_name),
                functools.partial(write_to_bag, bag),
            )
        )
    progress.update(task_id, completed=0)
    messages = 0
    with progress:
        while True:
            size = sizeof_fmt(os.path.getsize("test.bag"))
            # rich.print(f"Current file size: {size}", end = "\r")
            progress.update(
                task_id, completed=os.path.getsize("test.bag"), messages=messages
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
