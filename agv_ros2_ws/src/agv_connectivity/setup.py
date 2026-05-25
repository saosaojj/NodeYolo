import os
import glob
from setuptools import find_packages, setup

package_name = 'agv_connectivity'

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
    description='ROS2 WiFi and Bluetooth connectivity management package for AGV systems',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'wifi_manager_node = agv_connectivity.wifi_manager_node:main',
            'bluetooth_manager_node = agv_connectivity.bluetooth_manager_node:main',
        ],
    },
)
