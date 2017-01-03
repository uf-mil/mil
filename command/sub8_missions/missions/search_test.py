from txros import util
import numpy as np
from sub8.sub_singleton import Searcher


@util.cancellableInlineCallbacks
def run(sub_singleton):
    print "Starting"
    pattern = [np.array([-1, 0, 0])]
    s = Searcher(sub_singleton, sub_singleton.channel_marker.get_2d, pattern)
    resp = yield s.start_search(loop=False)
    print resp
    
    print "Done!"

