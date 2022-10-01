import asyncio
import functools
import importlib
import os
import re
import subprocess
import time
import traceback

import rich
import rich_click as click
import rosbag
import rospy
from rich.console import Console
from rich.live import Live
from rich.panel import Panel
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
from rich.table import Table


@click.group()
def bag():
    """
    Manage building of MIL documentation
    """
    pass


def validate_name(ctx, param, values):
    try:
        topic_names = [t[0] for t in rospy.get_published_topics()]
    except ConnectionRefusedError:
        raise click.ClickException(
            "Cannot connect to ROS. Please ensure that ROS is running."
        )

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
        return min, sec
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
    "--perf",
    help="Use performance mode (~40% faster, less control/readability, not needed in most cases)",
    default=False,
    required=False,
    is_flag=True,
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
    default="50GB",
    help="A limit on the recorded bag. Format as xx{MB,GB}; for example, 50MB or 4.1GB. Defaults to 50GB.",
)
def record(filename, names, all, perf, timeout, filesize):
    """
    Record the output of a list of topics named NAMES to a bag named FILENAME.
    """
    # If performance is desired, use original rosbag record functionality
    if perf:
        cmd = f"rosbag record -O {filename} {' '.join([n[0] for n in names])}"
        proc = subprocess.Popen(
            cmd.split(" "), stdout=subprocess.PIPE, stderr=subprocess.PIPE
        )
        print("Recording has begun!")
        orig_time = time.time()
        try:
            while True:
                try:
                    size = os.path.getsize(f"{filename}.active")
                except FileNotFoundError:
                    size = 0
                    orig_time = time.time()
                rich.print(
                    f"Recording in the background using performant mode... Current size: [chartreuse1 bold]{sizeof_fmt(size)}[/]",
                    end="\r",
                )
                if size > filesize:
                    print()
                    print("Cancelling recording because of filesize limit...")
                    proc.terminate()
                    print("Cancelled!")
                    break
                if timeout is not None and time.time() - orig_time > (
                    timeout[0] * 60 + timeout[1]
                ):
                    print()
                    print("Cancelling recording because of timeout limit...")
                    proc.terminate()
                    print("Cancelled!")
                    break
        except KeyboardInterrupt:
            print()
            print("Cancelling recording because of CTRL-C...")
            proc.terminate()
            print("Cancelled!")
        except Exception as e:
            print()
            print(f"Cancelling due to exception:")
            traceback.print_exc()
            proc.terminate()
            print("Cancelled!")
        return

    bag = rosbag.Bag(filename, "w")

    overall_progress = Progress(
        TextColumn(f"[bold blue]{filename}", justify="right"),
        BarColumn(),
        "[progress.percentage]{task.percentage:>3.1f}%",
        "•",
        FileSizeColumn(),
        "•",
        TextColumn("[chartreuse1]{task.fields[messages]} messages"),
        "•",
        TransferSpeedColumn(),
        "•",
        TimeElapsedColumn(),
        transient=True,
    )
    job_progress = Progress(
        TextColumn(
            "[chartreuse1 bold]{task.fields[topic_name]}[/] ([turquoise2]{task.fields[msg_name]}[/])",
            justify="right",
        ),
        BarColumn(),
        "[progress.percentage]{task.percentage:>3.1f}%",
        "•",
        TextColumn("[chartreuse1]{task.completed} messages"),
    )

    overall_task = overall_progress.add_task(filename, total=filesize, messages=0)

    progress_table = Table.grid()
    progress_table.add_row(overall_progress)
    progress_table.add_row(job_progress)

    trackers = {}  # topic_name: (task_id, msg_count)

    is_shutdown = False

    def write_to_bag(bag, topic_name, msg):
        try:
            if not is_shutdown:
                try:
                    bag.write(topic_name, msg, msg.header.stamp)
                except AttributeError:
                    bag.write(topic_name, msg)

                nonlocal trackers
                trackers[topic_name][1] += 1
        except:  # Avoid showing writing issues, breaking output
            pass

    subs = []
    rospy.init_node("mil_cli_listener", anonymous=True)
    for topic_name, msg_name in names:
        task_id = job_progress.add_task(
            "download", topic_name=topic_name, msg_name=msg_name, messages=0, start=True
        )
        trackers[topic_name] = [task_id, 0]
        msg_mod, class_name = msg_name.split("/")
        ros_pkg = importlib.import_module(".msg", package=msg_mod)
        subs.append(
            rospy.Subscriber(
                topic_name,
                getattr(ros_pkg, class_name),
                functools.partial(write_to_bag, bag, topic_name),
            )
        )

    start_time = time.time()
    with Live(progress_table, refresh_per_second=10):
        while not overall_progress.finished and (
            timeout is None or time.time() - start_time < (timeout[0] * 60 + timeout[1])
        ):
            size = sizeof_fmt(os.path.getsize(filename))
            total_msgs = sum(v[1] for v in trackers.values())
            overall_progress.update(
                overall_task, completed=os.path.getsize("test.bag"), messages=total_msgs
            )
            for topic_name, v in trackers.items():
                task_id, msg_count = v
                job_progress.update(task_id, completed=msg_count, total=total_msgs)

    console = Console()
    with console.status(f"[violet]Attempting to close up bag..."):
        for sub in subs:
            sub.unregister()

        bag.close()

    rich.print("[chartreuse1]All done!")


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
    try:
        while True:
            pass
    except KeyboardInterrupt:
        print("interrupted...")
    click.echo(f"{input_bag, input_topic, output_bag, output_topic}")
