#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['mbot_gazebo_control_ros'],
    package_dir={'mbot_gazebo_control_ros': 'ros/src/mbot_gazebo_control_ros'}
)

setup(**d)
