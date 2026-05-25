import os
import glob
from setuptools import find_packages, setup

package_name = 'agv_power_manager'

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
    description='AGV battery monitoring, charging control, power distribution, and low-power strategies',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'power_manager_node = agv_power_manager.power_manager_node:main',
            'battery_monitor_node = agv_power_manager.battery_monitor_node:main',
            'charging_controller_node = agv_power_manager.charging_controller_node:main',
        ],
    },
)
