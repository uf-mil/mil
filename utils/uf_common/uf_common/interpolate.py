from __future__ import division

def sample_curve((xs, ys), x):
    assert len(xs) == len(ys)
    # assert that xs is sorted?
    if x < xs[0]: x = xs[0]
    if x > xs[-1]: x = xs[-1]
    pairs = zip(xs, ys)
    # using bisect would be faster
    for (left_x, left_y), (right_x, right_y) in zip(pairs[:-1], pairs[1:]):
        if left_x <= x <= right_x:
            a = (x - left_x)/(right_x - left_x)
            return a * (right_y - left_y) + left_y
    assert False
