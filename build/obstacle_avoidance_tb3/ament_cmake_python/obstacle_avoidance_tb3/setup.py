from setuptools import find_packages
from setuptools import setup

setup(
    name='obstacle_avoidance_tb3',
    version='0.0.0',
    packages=find_packages(
        include=('obstacle_avoidance_tb3', 'obstacle_avoidance_tb3.*')),
)
