from setuptools import find_packages, setup

package_name = 'meeseeks'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
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
            'move_straight = meeseeks.move_straight:main',
            'move_straight2 = meeseeks.move_straight2:main',
            'five_star = meeseeks.five_star:main',
            'five_star2 = meeseeks.five_star2:main'
        ],
    },
)
