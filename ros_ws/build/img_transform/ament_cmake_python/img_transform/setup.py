from setuptools import find_packages
from setuptools import setup

setup(
    name='img_transform',
    version='0.0.0',
    packages=find_packages(
        include=('img_transform', 'img_transform.*')),
)
