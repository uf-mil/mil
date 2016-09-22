# flake8: noqa
import os
for module in os.listdir(os.path.dirname(__file__)):
    if module[0] == '_' or module[-3:] != '.py':
        continue
    __import__(module[:-3], locals(), globals())
del module
del os
