from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

# fetch values from package.xml
setup_args = generate_distutils_setup(
    packages=['thrust_mapper'],
    package_dir={'': 'src'},
    requires=[],
)

setup(**setup_args)