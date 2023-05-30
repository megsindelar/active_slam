from setuptools import find_packages
from setuptools import setup

setup(
    name='turtlebot_control',
    version='0.0.0',
    packages=find_packages(
        include=('turtlebot_control', 'turtlebot_control.*')),
)
