from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

SETUP_ARGS = generate_distutils_setup(
    packages=['gazebo_server'], package_dir={'': 'src'})

setup(**SETUP_ARGS)
