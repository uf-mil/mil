from typing import Callable, Optional


class Colors:
    red = "\033[31m"
    green = "\033[32m"
    yellow = "\033[33m"
    blue = "\033[34m"
    purple = "\033[35m"
    cyan = "\033[36m"
    white = "\033[37m"

    underline = "\033[2m"
    bold = "\033[1m"
    negative = "\033[3m"

    reset = "\033[0m"

    def __getattr__(self, arg):
        # If we get a non existent color, return the reset color
        return self.reset


class Printer:
    def __init__(self, string: Optional[str] = "", autospace: bool = False):
        self._autospace = autospace
        self._string = string

        # Default adding text
        self.text = lambda text: self + text

        # Colors
        self.red = lambda text: self + (Colors.red + str(text) + Colors.reset)
        self.green = lambda text: self + (Colors.green + str(text) + Colors.reset)
        self.yellow = lambda text: self + (Colors.yellow + str(text) + Colors.reset)
        self.blue = lambda text: self + (Colors.blue + str(text) + Colors.reset)
        self.purple = lambda text: self + (Colors.purple + str(text) + Colors.reset)
        self.cyan = lambda text: self + (Colors.cyan + str(text) + Colors.reset)
        self.white = lambda text: self + (Colors.white + str(text) + Colors.reset)

        # Text effects
        self.underline = lambda text: Printer(
            self._string + Colors.underline + str(text) + Colors.reset,
        )
        self.bold = lambda text: Printer(
            self._string + Colors.bold + str(text) + Colors.reset,
        )
        self.negative = lambda text: Printer(
            self._string + Colors.negative + str(text) + Colors.reset,
        )

        # For passing in custom formatting
        self.custom = lambda text, effect: self + (effect + str(text) + Colors.reset)

    def __repr__(self):
        return self._string + Colors.reset

    __str__ = __repr__

    def __add__(self, other):
        extra_space = " " if self._autospace and self._string != "" else ""
        return Printer(self._string + extra_space + str(other), self._autospace)

    @property
    def set_red(self):
        return Printer(self._string + Colors.red)

    @property
    def set_green(self):
        return Printer(self._string + Colors.green)

    @property
    def set_yellow(self):
        return Printer(self._string + Colors.yellow)

    @property
    def set_blue(self):
        return Printer(self._string + Colors.blue)

    @property
    def set_purple(self):
        return Printer(self._string + Colors.purple)

    @property
    def set_cyan(self):
        return Printer(self._string + Colors.cyan)

    @property
    def set_white(self):
        return Printer(self._string + Colors.white)

    @property
    def reset(self):
        return Printer(self._string + Colors.reset)

    def space(self, count=1):
        return Printer(self._string + " " * count)

    def newline(self, count=1):
        return Printer(self._string + "\n" * count)

    def enable_autospaces(self):
        self._autospace = False

    def disable_autospaces(self):
        self._autospace = True


class FprintFactory:
    """
    Factory method for producing a printer with the specified characteristics.

    Args:
        title (Optional[str]): The title to produce with each printed
            message.
        time (Optional[Callable]): A method for getting the time to produce
            with each method. If ``None``, then no time is sent with a message.
        msg_color (Optional[str]): The color of each message. If ``None``,
            defaults to white.
        auto_bold (bool): Automatically bolds each method. Defaults to ``True``.
        newline (int): The number of newlines to print after each method.
    """

    def __init__(
        self,
        title: Optional[str] = None,
        time: Optional[Callable] = None,
        msg_color: Optional[str] = None,
        auto_bold: bool = True,
        newline: int = 1,
    ):
        assert time is None or callable(
            time,
        ), "`time` should be `None` for no printing or a function that generates a timestamp."
        assert msg_color is None or isinstance(
            msg_color,
            str,
        ), "`msg_color` should be `None` for default printing or a string color."
        assert isinstance(
            auto_bold,
            bool,
        ), "`auto_bold` should be true or false if messages should be printed\
                               as bold by default or not"
        assert newline is None or isinstance(
            newline,
            int,
        ), "`newline` should be the number of newlines after the text (default 1)"

        # All these can be overwritten if not specified here
        self.title = title  # Title to print with each message
        # Either `None` for no printing or a function that generates a
        # timestamp
        self.time = time
        self.msg_color = (
            msg_color  # Either `None` for default printing or a string color
        )
        self.auto_bold = auto_bold  # Should each message be bolded by default
        self.newline = newline  # The number of newlines characters to add to the end

        self.printer = Printer()

    def fprint(self, text: str, **kwargs):
        """
        Prints some text with the specified characteristics. Characteristics
        can be passed through the kwargs argument or through the class' constructor.

        Args:
            text (str): The text to format and then print
            kwargs: Any characteristics to print with the text. All keyword arguments
              are the same as the arguments specified in the constructor.
        """
        title = kwargs.get("title", self.title)
        time = kwargs.get("time", self.time)
        msg_color = kwargs.get("msg_color", self.msg_color)
        auto_bold = kwargs.get("auto_bold", self.auto_bold)
        newline = kwargs.get("newline", self.newline)

        message = self.printer
        if title is not None:
            message = message.set_blue.bold(title).reset.space()

        if time is not None:
            t = time()
            message = message.bold(t).space()

        message += ": "

        if auto_bold:
            text = str(self.printer.bold(text))

        if msg_color is not None:
            message = message.custom(text, getattr(Colors, msg_color))
        else:
            message = message.text(text)

        if newline == 1:
            print(message)
        else:
            print(message.newline(newline - 1))


# Standard instantiation
fprint = FprintFactory().fprint
