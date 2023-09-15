#! /usr/bin/env python3
# ! DO NOT MANUALLY INVOKE THIS setup.py, USE CATKIN INSTEAD

from catkin_pkg.python_setup import generate_distutils_setup
from setuptools import setup

# Fetch values from package.xml
setup_args = generate_distutils_setup(
    packages=["navigator_path_planner", "lqrrt"],
    package_dir={"lqrrt": "lqRRT/lqrrt"},
)

setup(**setup_args)
