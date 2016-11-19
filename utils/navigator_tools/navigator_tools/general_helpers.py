import rospy


class MissingPerceptionObject():
    def __init__(self, name):
        self.name = name

class Colors():
    # Some cool stuff could happen here
    red = '\033[91m'
    green = '\033[92m'
    yellow = '\033[93m'
    blue = '\033[94m'
    bold = '\033[1m'

    reset = '\033[0m'

    def __getattr__(self, arg):
        # If we get a non existent color, return the reset color
        return self.reset

class Seperator():
    # TODO
    def __getattr__(self, *args, **kwargs):
        return

    def equals(self, _len, blank_space=False):
        text = "{}".format("=" * len)
        if blank_space:
            return "\n{}\n".format(text)

        return text


def fprint(msg, time=None, title=None, newline=True, msg_color=None):
    time_header = False
    title_header = False

    if title is not None:
        title_header = "{C.blue}{C.bold}{title}{C.reset}".format(C=Colors, title=title)

    if msg_color is not None:
        msg = "{color}{msg}{C.reset}".format(color=getattr(Colors(), msg_color), C=Colors, msg=msg)

    if time == "":
        time_header = False

    elif time is None:
        try:
            time = rospy.Time.now().to_sec()
            time_header = "{C.bold}{time}{C.reset}".format(C=Colors, time=time)
        except rospy.exceptions.ROSInitException:
            pass
    else:
        time_header = "{C.bold}{time}{C.reset}".format(C=Colors, time=time)

    if title_header and time_header:
        # (TIME) HEADER: message
        to_print = "{time} {title}: {msg}"
    elif time_header:
        # (TIME): message
        to_print = "{time}: {msg}"
    elif title_header:
        # HEADER: message
        to_print = "{title}: {msg}"
    else:
        # message
        to_print = "{msg}"

    if newline:
        if not isinstance(newline, bool):
            msg += "\n" * (newline - 1)  # Since printing implicitly adds a new line
        print to_print.format(time=time_header, title=title_header, msg=msg)
    else:
        # Note, this adds a space at the end.
        print to_print.format(time=time_header, title=title_header, msg=msg),

print_t = fprint  # For legacy
