import abc


class TestUnit:
    __metaclass__ = abc.ABCMeta

    @abc.abstractmethod
    def create_spoofs(self):
        return

    @abc.abstractmethod
    def run_tests(self):
        return
