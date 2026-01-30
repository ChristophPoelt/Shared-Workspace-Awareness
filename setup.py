from setuptools import find_packages, setup

import os
from glob import glob

package_name = 'meeseeks'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/meeseeks"]),
        ("share/meeseeks", ["package.xml"]),
        (os.path.join("share", "meeseeks", "launch"), glob("launch/*.launch.py")),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='phri1',
    maintainer_email='phri1@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'angle_calculation = angle_calculation:main',
            'globalVariables = globalVariables:main',
            'robot_gestures = meeseeks.robot_gestures:main',
            'main_logic = meeseeks.main_logic:main',
            'pointing_to_target_logic = pointing_to_target_logic:main',
            'robot_initialization = robot_initialization:main',
            'targetSelection = targetSelection:main'
        ],
    },
)
