class Cache(object):
    """No support for **kwargs**"""
    def __init__(self, func):
        self.call_dict = {}
        self.func = func

    def __call__(self, *args):
        if args not in self.call_dict.keys():
            result = self.func(*args)
            self.call_dict[args] = result
        else:
            result = self.call_dict[args]
        return result


@Cache
def add(a, b):
    print 'adding', a, b
    return a + b


@Cache
def gooberstein(data):
    print "Being called"
    return data + "balls"


if __name__ == '__main__':

    print add(1, 2)
    print add(1, 2)

    print add(2, 3)
    print add(2, 3)

    print gooberstein("hello")
    print gooberstein("hello")
    print gooberstein("hello")
