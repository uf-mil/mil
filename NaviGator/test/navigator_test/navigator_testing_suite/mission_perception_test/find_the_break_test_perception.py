import txros
from twisted.internet import defer


class FindTheBreakTestPerception:
    def __init__(self, nh):
        self.nh = nh

    async def count_pipes(self):
        await self.nh.sleep(5)
        return 4
