#!/usr/bin/python3

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['effector_control', 'tf', 'tf2_ros'],
    scripts=['src/slider_service.py'],
    package_dir={'': 'src'}
)

setup(**d)
