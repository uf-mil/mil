#!/usr/bin/env python3

# Systems[]
#   Global variable that contains all the systems. Used to standardized file name. Ex Subjugator or Navigator

# writeTests(fileName, topicsToCheck[])
#   fileName should standardized, this input should be from a selected number of names
#   topicsToCheck[] this array should only contain valid topics that we will make sure data is in.
#       What ever program calls this function should only pass in valid topics. This will be a list of strings.
#   This should not delete old topicsToCheck only add new ones.

# deleteTests(fileName, topicsToCheck[])
#   fileName should standardized, this input should be from a selected number of names
#   topicsToCheck[] this array should only contain valid topics that we will make sure data is in.
#       What ever program calls this function should only pass in valid topics. This will be a list of strings.
#   delete tests from the file.

# readTests(filename)
#   reads and runs all the tests in the file.
#   returns whether each test pass or fail
import rospy
import rostopic


def writeTests(filename, topicsToCheck):
    tests = open(filename, "a+")
    lines = tests.readlines()

    # If the topic to add is not already in the list add it
    for topic in topicsToCheck:
        topic += "\n"
        if topic not in lines:
            tests.write(topic)
    tests.close()


def deleteTests(filename, topicsToCheck):
    tests = open(filename)
    lines = tests.readlines()
    tests.close()

    # If the topic to add is not already in the list add it
    for topic in topicsToCheck:
        topic += "\n"
        if topic in lines:
            lines.remove(topic)

    tests = open(filename, "w")
    tests.writelines(lines)


def readTests(filename):
    tests = open(filename)
    lines = tests.readlines()
    dataTypes = []
    for i in range(len(lines)):
        lines[i] = lines[i][:-1]
        TopicType, topic_str, _ = rostopic.get_topic_class(lines[i])
        dataTypes.append(TopicType)
        lines[i] = topic_str
        rospy.Subscriber(lines[i], dataTypes[i], some_callback)


def some_callback(msg):
    rospy.loginfo("Got the message: " + str(msg))

    # We need to call the ros topic and verify the data WIP


if __name__ == "__main__":
    rospy.init_node("topic_publisher_checker")
    writeTests("SubChecklist.txt", ["dvl", "odom"])
    deleteTests("SubChecklist.txt", ["testing"])
    readTests("SubChecklist.txt")
