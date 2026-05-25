from setuptools import setup

setup(
    name='agv_utils',
    version='1.0.0',
    packages=['agv_utils'],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/agv_utils']),
        ('share/agv_utils/launch', ['launch/utils.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='AGV Team',
    maintainer_email='agv@example.com',
    description='Performance optimization utilities for AGV ROS2 project',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'performance_monitor = agv_utils.performance_monitor:main',
        ],
    },
)
