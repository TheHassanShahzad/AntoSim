from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'antobot_ant_bringup'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.rviz')),
        ('share/' + package_name + '/maps', glob('maps/*.*')),  # <-- this line added
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='the-hassan-shahzad',
    maintainer_email='hshahzad2005108277@gmail.com',
    description='Launch package for Antobot Ant robot bringup',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [],
    },
)
