# ! DO NOT MANUALLY INVOKE THIS setup.py, USE CATKIN INSTEAD

from catkin_pkg.python_setup import generate_distutils_setup
from setuptools import setup

# fetch values from package.xml
setup_args = generate_distutils_setup(
    packages=[
        "navigator_missions",
        "nav_missions_test",
        "nav_missions_lib",
        "vrx_missions",
    ],
)

setup(**setup_args)
