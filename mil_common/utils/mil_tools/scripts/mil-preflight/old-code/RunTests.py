import importlib


def getHardWareCheckist(robotName):
    robotName += ".hardware"
    module = importlib.import_module(robotName)

    if hasattr(module, "hardwareTests"):
        return module.hardwareTests
    else:
        return []
