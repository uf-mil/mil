from __future__ import division

import math

class V(tuple):
    for name, operator in [("neg", "-%s"), ("pos", "+%s"), ("abs", "abs(%s)")]:
        exec("def __%s__(self): return V(%s for x in self)" % (name, operator % "x"))
    for name, operator in [
        ("add", "%s+%s"), ("sub", "%s-%s"), ("mul", "%s*%s"), ("truediv", "%s/%s"), ("floordiv", "%s//%s"),
        ("call", "%s(%s)"),
    ]:
        exec("""def __%s__(self, other):
        try:
            return %s(%s for x, y in zip(self, other))
        except:
            return V(%s for x in self)""" % (name, "sum" if name == "mul" else "V", operator % ("x", "y"), operator % ("x", "other")))
        exec("""def __r%s__(self, other):
        try:
            return %s(%s for x, y in zip(self, other))
        except:
            return V(%s for x in self)""" % (name, "sum" if name == "mul" else "V", operator % ("y", "x"), operator % ("other", "x")))
    def __mod__(self, other):
        if len(self) == 3 and len(other) == 3:
            (x, y, z), (X, Y, Z) = self, other
            return V([y*Z-z*Y, z*X-x*Z, x*Y-y*X])
        else:
            return V((
                other[0] * self[0] - other[1] * self[1] - other[2] * self[2] -  other[3] * self[3],
                other[1] * self[0] + other[0] * self[1] + other[3] * self[2] -  other[2] * self[3],
                other[2] * self[0] - other[3] * self[1] + other[0] * self[2] +  other[1] * self[3],
                other[3] * self[0] + other[2] * self[1] - other[1] * self[2] +  other[0] * self[3],
            ))
    def __rmod__(self, other):
        return self.__mod__.im_func(other, self)
    def __repr__(self):
        return 'v%s' % tuple.__repr__(self)
    #def __getitem__(self, item):
    #    if isinstance(item, slice):
    #        return V(tuple.__getitem__(self, item))
    #    else:
    #        return tuple.__getitem__(self, item)
    #def __getslice__(self, i, j):
    #    return self.__getitem__(slice(i, j))
    def mag(self):
        return (self*self)**.5
    def mag2(self):
        return self*self
    def unit(self):
        try:
            return self/self.mag()
        except ZeroDivisionError:
            return v(1, 0, 0)
    def conj(self):
        return V([self[0]] + list(-V(self[1:])))
    def quat_to_matrix(self):
        return (
            (1 - 2*self[2]*self[2] - 2*self[3]*self[3],     2*self[1]*self[2] - 2*self[3]*self[0],     2*self[1]*self[3] + 2*self[2]*self[0]),
            (    2*self[1]*self[2] + 2*self[3]*self[0], 1 - 2*self[1]*self[1] - 2*self[3]*self[3],     2*self[2]*self[3] - 2*self[1]*self[0]),
            (    2*self[1]*self[3] - 2*self[2]*self[0],     2*self[2]*self[3] + 2*self[1]*self[0], 1 - 2*self[1]*self[1] - 2*self[2]*self[2]),
        )
    def quat_to_matrix4(self):
        return [list(row)+[0] for row in self.quat_to_matrix()] + [[0, 0, 0, 1]]
    def quat_to_axisangle(self):
        self = self.unit()
        if self[0] < 0:
            self = -self
        return V(self[1:]).unit(), math.acos(self[0]) * 2
    def quat_to_scaledaxis(self):
        axis, angle = self.quat_to_axisangle()
        return axis * angle
    def quat_rot(self, v):
        return V((self % V([0] + list(v)) % self.conj())[1:])/self.mag2()
    def scale(self, other):
        return V(x*y for x, y in zip(self, other))


def matrix_to_quat(m1):
    w = math.sqrt(1 + m1[0][0] + m1[1][1] + m1[2][2]) / 2
    w4 = 4 * w
    return v(
        w,
        (m1[2][1] - m1[1][2]) / w4,
        (m1[0][2] - m1[2][0]) / w4,
        (m1[1][0] - m1[0][1]) / w4,
    )


def axisangle_to_quat(axis, angle):
    return V([math.cos(angle/2)] + list(math.sin(angle/2) * V(axis).unit()))

def v(*args):
    return V(args)

if __name__ == "__main__":
    q = v(1, 0, 0, 0)
    s = v(1, .1, 0, 0).unit()
    for i in xrange(10):
        print q.quat_to_matrix()
        q = q.quat_mul(s)
    print q[0]
