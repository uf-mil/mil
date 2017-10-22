from argparse import ArgumentParser


class ArgumentParserException(Exception):
    def __init__(self, message):
        self.message = message

    def __str__(self):
        return 'Error parsing arguments: ' + self.message


class ThrowingArgumentParser(ArgumentParser):
    '''
    Extension of standard ArgumentParser that will throw an exception
    if a parse error is encountered so programs can handle this error
    in custom ways besides just printing the help screen and exiting.

    https://stackoverflow.com/questions/14728376/i-want-python-argparse-to-throw-an-exception-rather-than-usage

    '''
    def error(self, message):
        raise ArgumentParserException(message)
