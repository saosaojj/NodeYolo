import os
from glob import glob
from setuptools import setup

setup(
    name='agv_video_stream',
    version='1.0.0',
    packages=['agv_video_stream'],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/agv_video_stream']),
        ('share/agv_video_stream', ['package.xml']),
        (os.path.join('share', 'agv_video_stream', 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', 'agv_video_stream', 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='AGV Team',
    maintainer_email='agv@example.com',
    description='ROS2 video streaming and recording package for AGV',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'video_server_node = agv_video_stream.video_server_node:main',
            'video_recorder_node = agv_video_stream.video_recorder_node:main',
        ],
    },
)
