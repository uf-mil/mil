from argparse import ArgumentParser


class ArgumentParserException(Exception):
    def __init__(self, message: str):
        self.message = message

    def __str__(self):
        return f"Error parsing arguments: {self.message}"


class ThrowingArgumentParser(ArgumentParser):
    """
    Extension of standard :class:`argparse.ArgumentParser` that will throw an exception
    if a parse error is encountered so programs can handle this error
    in custom ways besides just printing the help screen and exiting.

    The error method of the argument parser class is overloaded such that it
    will raise :class:`ArgumentParserException`.
    """

    # From: https://stackoverflow.com/questions/14728376/i-want-python-argparse-to-throw-an-exception-rather-than-usage
    def error(self, message: str):
        """
        Handles errors in the ArgumentParser. Only raises the :class:`ArgumentParserException`
        and does nothing else.
        """
        raise ArgumentParserException(message)
