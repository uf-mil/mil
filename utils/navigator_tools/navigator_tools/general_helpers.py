import rospy

def print_t(_str, time=None, title=None):
    time_header = None
    title_header = False

    if title is not None:
        title_header = "\033[94m\033[1m{}\033[0m".format(title)

    if time is None:
        try:
            time = rospy.Time.now().to_sec()
            time_header = "\033[1m{}\033[0m".format(time)
        except rospy.exceptions.ROSInitException:
            pass
    else:
        time_header = "\033[1m{}\033[0m".format(time)

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

    print to_print.format(time=time_header, title=title_header, msg=_str)