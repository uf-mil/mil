#!/usr/bin/env python3

import contextlib

import rospy
import rostopic


def writeTests(filename, testsToAdd):
    hardwareLines = []
    softwareLines = []
    try:
        hardwareLines = getHardwareChecks(filename)
        softwareLines = getSoftwareChecks(filename)
    except OSError:
        pass

    tests = open(filename, "w")
    tests.write("=== Hardware Checks ===\n")

    # If the hardware topic to add is not already in the list add it
    for test in testsToAdd:
        if test[0] == "h":
            test = test[2:]
            test += "\n"
            if test not in hardwareLines:
                tests.write(test)

    tests.write("=== Software Checks ===\n")

    # If the software topic to add is not already in the list add it
    for test in testsToAdd:
        if test[0] == "s":
            test = test[2:]
            test += "\n"
            if test not in softwareLines:
                tests.write(test)

    tests.close()


def deleteTests(filename, testsToDelete):
    hardwareLines = []
    softwareLines = []

    try:
        hardwareLines = getHardwareChecks(filename)
        softwareLines = getSoftwareChecks(filename)
    except OSError:
        return False

    for test in testsToDelete:
        test += "\n"
        with contextlib.suppress(Exception):
            hardwareLines.remove(test)

        with contextlib.suppress(Exception):
            softwareLines.remove(test)

    tests = open(filename, "w")
    tests.write("=== Hardware Checks ===\n")
    tests.writelines(hardwareLines)
    tests.write("=== Software Checks ===\n")
    tests.writelines(softwareLines)


def readTests(filename):
    return [getHardwareChecks(filename), getSoftwareChecks(filename)]


def runTests(filename, index):
    sChecks = getSoftwareChecks(filename)

    testType = sChecks[index].split()[0]
    testTopic = sChecks[index].split()[1]

    if testType == "TOPIC":
        TopicType, topic_str, _ = rostopic.get_topic_class(testTopic)

        try:
            rospy.wait_for_message(topic_str, TopicType)
            return True
        except Exception:
            return False


def getHardwareChecks(filename):
    checks = open(filename)
    foundHardware = False
    hardwareChecks = []
    for index, line in enumerate(checks):
        if "=== Hardware Checks ===" in line:
            foundHardware = True
            continue

        if "=== Software Checks ===" in line:
            foundHardware = False
            continue

        if foundHardware:
            hardwareChecks.append(line)

    checks.close()
    return hardwareChecks


def getSoftwareChecks(filename):
    checks = open(filename)
    foundSoftware = False
    softwareChecks = []
    for index, line in enumerate(checks):
        if "=== Hardware Checks ===" in line:
            foundSoftware = False
            continue

        if "=== Software Checks ===" in line:
            foundSoftware = True
            continue

        if foundSoftware:
            softwareChecks.append(line)

    checks.close()
    return softwareChecks
