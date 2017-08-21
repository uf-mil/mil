# flake8: noqa
import os
import rospkg
from mil_misc_tools.text_effects import fprint

# Had to use rospkg, os.path.dirname(__file__) wasn't working
for module in os.listdir(os.path.join(rospkg.RosPack().get_path("navigator_test"), 'navigator_tests/')):
    if module[0] == '_' or module[-3:] != '.py':
        continue
    try:
        __import__(module[:-3], locals(), globals())
    except Exception as e:
        fprint("ERROR in module: {}".format(module), msg_color='red')
        print e, '\n'

del fprint
del rospkg
del module
del os
