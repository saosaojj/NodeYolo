import os
import glob
from setuptools import find_packages, setup

package_name = 'agv_navigation'

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
        (os.path.join('share', package_name, 'maps'), glob.glob('maps/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='todo',
    maintainer_email='todo@todo.com',
    description='AGV autonomous navigation, path planning, and driving control package',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'agv_controller_node = agv_navigation.agv_controller_node:main',
            'agv_navigator_node = agv_navigation.agv_navigator_node:main',
            'agv_odometry_node = agv_navigation.agv_odometry_node:main',
        ],
    },
)
