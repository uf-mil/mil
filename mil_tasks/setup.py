# ! DO NOT MANUALLY INVOKE THIS setup.py, USE CATKIN INSTEAD

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

# Fetch values from package.xml
setup_args = generate_distutils_setup(
    packages=['mil_tasks_core', 'mil_tasks_examples', 'mil_tasks_gui'],
)

setup(**setup_args)
