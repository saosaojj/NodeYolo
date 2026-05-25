import os
import glob
from setuptools import find_packages, setup

package_name = 'agv_io_controller'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob.glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob.glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='todo',
    maintainer_email='todo@todo.com',
    description='ROS2 IO controller package for digital and analog IO point control on AGV systems',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'io_controller_node = agv_io_controller.io_controller_node:main',
            'io_simulator_node = agv_io_controller.io_simulator_node:main',
        ],
    },
)
