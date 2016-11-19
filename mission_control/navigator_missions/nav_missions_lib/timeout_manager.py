import numpy as np


class TimeoutManager(object):

    @classmethod
    def generate_timeouts(cls, time_left, missions_left):
        weights = np.array([x.weight for x in missions_left], dtype=np.float32)
        total = sum(weights)
        weights = np.divide(weights, total)
        # print total
        # print weights
        # print time_left
        for i, m in enumerate(missions_left):
            m.timeout = weights[i] * time_left

    @classmethod
    def can_repeat(cls, missions_left, time_left, mission):
        if mission.attempts >= 2:
            return False
        total_min_time = sum(np.array([x.min_time for x in missions_left]))
        if total_min_time >= time_left:
            return False
        return True
