import os
import glob
from setuptools import find_packages, setup

package_name = 'agv_plc_bridge'

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
    description='ROS2 PLC/ModbusTCP bridge package for communication with mainstream PLCs',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'modbus_tcp_node = agv_plc_bridge.modbus_tcp_node:main',
            'plc_manager_node = agv_plc_bridge.plc_manager_node:main',
            'plc_simulator = agv_plc_bridge.plc_simulator:main',
        ],
    },
)
