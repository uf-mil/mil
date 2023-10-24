import genpy
from axros import NodeHandle


class SpoofPubilsher:
    """
    Attributes:
    topic_name (str): the name of the topic
    message_type (str): the actual content 
    responses (List[str]): stores a list of responses
    times (int): represents the current time
    """
    def __init__(self, topic_name, message_type, responses, times):
        self.topic_name = topic_name
        self.responses = responses
        self.times = times
        self.message_type = message_type
        self.total = len(self.responses)

    async def start(self, nh):
        """
        To simplify, this publishes the necessary information within an allocation time slot. 

        Args:
            nh (NodeHandle): peforms all operations for the successful publication of content. 
        """
        self.my_pub = await nh.advertise(self.topic_name, self.message_type)
        i = 0
        started = nh.get_time()
        while True:
            if len(self.times) == 0:
                break
            if nh.get_time() - started > genpy.Duration(self.times[i % self.total]):
                i += 1
                started = nh.get_time()
            self.my_pub.publish(self.responses[i % self.total])
            await nh.sleep(0.3)

    async def stop(self):
        """
        Stops the process that is initally set by "nh.advertise". 
        """
        await self.my_pub.shutdown()


class SpoofService:
    """
    Attributes:
    service_name (str): the name of the service
    responses (List[str]): stores a list of responses
    message_type (str): the actual content 
    total (int): the length of the array "responses"
    count (int): a regular incrementor
    serv: performs service functions, such as "setup" and "shutdown" 
    """
    def __init__(self, service_name, message_type, responses):
        self.service_name = service_name
        self.responses = responses
        self.message_type = message_type
        self.total = len(responses)
        self.count = 0
        self.serv = None

    async def start(self, nh: NodeHandle):
        """sets up the process using a NodeHandler 

        Args:
            nh (NodeHandle): it is an object that ensures a proper setup using service components
        """
        self.serv = nh.advertise_service(
            self.service_name, self.message_type, self._service_cb
        )
        await self.serv.setup()

    def _service_cb(self, req):
        """Increments the counter to move on to the next index position

        Args:
            req (n/a): unused variable 

        Returns:
            ans: it returns the value from array "responses" from the incremented index
        """
        ans = self.responses[self.count % self.total]
        self.count += 1
        return ans

    async def stop(self):
        """
        Stops the process that is initally set by "nh.advertise_service". 
        """
        await self.serv.shutdown()


class SpoofGenerator:
    def spoof_service(self, service_name, message_type, responses):
        """
        Returns "SpoofService" object

        Args:
            service_name (str): the name of the service
            message_type (str): the actual content 
            responses (List[str]): stores a list of responses

        Returns:
            SpoofService: an instance of "SpoofService" with arguments passed from the parameters of the function
        """
        return SpoofService(service_name, message_type, responses)

    def spoof_publisher(self, topic_name, message_type, responses, time):
        """
        Returns "SpoofPubilsher" object

        Args:
            topic_name (str): the name of the topic
            message_type (str): the actual content
            responses (List[str]): stores a list of responses
            time (int): represents the current time

        Returns:
            SpoofPubilsher: an instance of "SpoofPubilsher" with arguments passed from the parameters of the function
        """
        return SpoofPubilsher(topic_name, message_type, responses, time)
