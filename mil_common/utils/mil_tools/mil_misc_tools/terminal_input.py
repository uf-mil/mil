#!/usr/bin/env python3
import sys
import tty
import termios

__author__ = "David Soto"


def get_ch():
    """
    Gets a single character from stdin. Doesn't echo to screen. Catches ``CTRL-C`` 
    and ``CTRL-E`` and raises exceptions.

    Raises:
        KeyboardInterrupt: The user presses ``CTRL-C``
        EOFError: The user presses ``CTRL-D`` character (the ``EOF`` characetr)
    """
    # Adapted from
    # http://stackoverflow.com/questions/510357/python-read-a-single-character-from-the-user/21659588#21659588
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)

    # Handle signal keys
    if ch == "\x03":  # CTRL-C
        raise KeyboardInterrupt
    if ch == "\x04":  # CTRL-D
        raise EOFError("Got EOF character")
    return ch
