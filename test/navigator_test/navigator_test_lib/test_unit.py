import abc


class TestUnit(object):
    __metaclass__ = abc.ABCMeta

    @abc.abstractmethod
    def create_spoofs(self):
        return

    @abc.abstractmethod
    def run_tests(self):
        return
