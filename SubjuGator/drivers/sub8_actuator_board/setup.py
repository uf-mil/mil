# ! DO NOT MANUALLY INVOKE THIS setup.py, USE CATKIN INSTEAD

from catkin_pkg.python_setup import generate_distutils_setup
from setuptools import setup

# Fetch values from package.xml
setup_args = generate_distutils_setup(packages=["sub8_actuator_board"])

setup(**setup_args)
