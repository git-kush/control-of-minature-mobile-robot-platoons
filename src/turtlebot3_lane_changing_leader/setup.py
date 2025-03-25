from setuptools import setup
import os
from glob import glob

package_name = 'turtlebot3_lane_changing_leader'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*launch.[pxy][yma]*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',  # Replace with your name
    maintainer_email='your.email@example.com',  # Replace with your email
    description='Lane changing leader node for TurtleBot3 using potential field approach',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'lane_changing_leader = turtlebot3_lane_changing_leader.lane_changing_leader:main'
        ],
    },
)