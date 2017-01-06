from txros import util


@util.cancellableInlineCallbacks
def run(sub_singleton):
    print "Square"
    square_length = 2  # meters
    yield sub_singleton.move.forward(square_length).go()
    yield sub_singleton.move.right(square_length).go()
    yield sub_singleton.move.backward(square_length).go()
    yield sub_singleton.move.left(square_length).go()
    print "Done!"
