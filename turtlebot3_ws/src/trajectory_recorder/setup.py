import os
from glob import glob
from setuptools import setup, find_packages

package_name = 'trajectory_recorder'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob(os.path.join('launch', '*.launch.py'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@example.com',
    description=(
        'Grabacion y reproduccion de trayectorias para TurtleBot3 '
        'con correccion PID basada en odometria.'
    ),
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'recorder_node = trajectory_recorder.recorder:main',
            'player_node = trajectory_recorder.player:main',
            'optimizer_node = trajectory_recorder.optimizer:main',
            'optimizer = trajectory_recorder.optimizer:main',
        ],
    },
)
