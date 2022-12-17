class SubjuGatorException(Exception):
    """
    Base class for all SubjuGator-related exceptions.
    """

    def __init__(self, message):
        super().__init__(message)


class KilledException(SubjuGatorException):
    """
    An action was attempted on SubjuGator, but it is killed
    """

    def __init__(self):
        super().__init__("SubjuGator is killed!")
