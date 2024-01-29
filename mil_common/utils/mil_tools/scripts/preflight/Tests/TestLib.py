import rospy
import rosservice
import rostopic


class Test:
    # Constructor
    def __init__(self, name, description, communicationType, address):
        self.name = name
        self.description = description
        self.communicationType = communicationType
        self.address = address
        self.needsHumanAuthorization = False

    # Check if the communication interface is up and running
    def isActive(self):
        # Check if the topic is up
        if self.communicationType == "Topic":
            topicType, topicStr, _ = rostopic.get_topic_class(
                self.address,
            )  # get topic class
            try:
                rospy.wait_for_message(
                    topicStr,
                    topicType,
                )  # try to get a message from that topic
                return True
            except Exception:
                return False

        # Check if the service is up
        if self.communicationType == "Service":
            return rosservice.waitForService(
                self.address,
            )  # Wait for the service to be up

        # Check if the action server is up
        if self.communicationType == "Action":
            # action client
            topicType, topicStr, _ = rostopic.get_topic_class(
                self.address + "/feedback",
            )
            try:
                # Wait for a message from the topic
                rospy.wait_for_message(topicStr, topicType)
                return True
            except Exception:
                return False

    # Send data over the communication interface and pass in the return values into the check function
    def VerifyData(self, args, checkFunction):
        result = ""

        if self.communicationType == "Topic":
            # Get the class of the topic
            topicType, topicStr, _ = rostopic.get_topic_class(self.address)
            try:
                # Get a message from the topic
                result = rospy.wait_for_message(topicStr, topicType)
            except Exception:
                return False

        if self.communicationType == "Service":
            try:
                # Get a message form the service
                result = rosservice.call_service(self.address, args)
            except Exception:
                return False

        if self.communicationType == "Action":
            # Get the class of the topic, remember an action will publish its results to a topic with /goal at the end
            topicType, topicStr, _ = rostopic.get_topic_class(
                self.address + "/feedback",
            )
            try:
                # Wait for a message from the topic
                result = rospy.wait_for_message(topicStr, topicType)
            except Exception:
                return False

        return checkFunction(args, result)

    def runCommand(self, args):
        # use a dictionary
        if self.communicationType == "Topic":
            topicType, topicStr, _ = rostopic.get_topic_class(self.address)
        if self.communicationType == "Service":
            # Do something
            self
        if self.communicationType == "Action":
            # Do something
            self
        return False
