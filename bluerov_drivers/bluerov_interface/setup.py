#!/usr/bin/env python3

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

# for your packages to be recognized by python
d = generate_distutils_setup(
 packages=['bluerov_interface'],
 package_dir={'bluerov_interface': 'bluerov_interface'}
)

setup(**d)
