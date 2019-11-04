# TODO: Give this folder a cooler name
# flake8: noqa
import os
import sys

for module in os.listdir(os.path.dirname(__file__)):
    if module == '__init__.py' or module[-3:] != '.py':
        continue
    # TODO: Something more sophisticated
    __import__(module[:-3], locals(), globals())

del module
del os
