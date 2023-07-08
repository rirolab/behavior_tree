#!/usr/bin/env python3
import os
from glob import glob
from setuptools import setup

package_name = 'behavior_tree'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name, 'params'), glob(os.path.join('params','*.yaml'))),                
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    author='Daehyung Park',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'static_bt = behavior_tree.static_behavior_tree:main',
            'dynamic_bt = behavior_tree.dynamic_behavior_tree:main',
            ],
    },
)

