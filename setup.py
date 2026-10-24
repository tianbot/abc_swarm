#!/usr/bin/env python3
from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

setup(**generate_distutils_setup(packages=['abc_swarm', 'simple_pid'], package_dir={'': 'src'}))
