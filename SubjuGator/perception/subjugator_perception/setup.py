# ! DO NOT MANUALLY INVOKE THIS setup.py, USE CATKIN INSTEAD

from catkin_pkg.python_setup import generate_distutils_setup
from setuptools import setup

# fetch values from package.xml
setup_args = generate_distutils_setup(
    packages=["subjugator_vision_tools", "subjugator_vision_tools.segmentation"],
)

setup(**setup_args)
