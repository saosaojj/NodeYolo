from setuptools import setup

setup(
    name='agv_path_planner',
    version='1.0.0',
    packages=['agv_path_planner'],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/agv_path_planner']),
        ('share/agv_path_planner', ['package.xml']),
        ('share/agv_path_planner/launch', ['launch/path_planner.launch.py']),
        ('share/agv_path_planner/config', ['config/planner_config.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='AGV Team',
    maintainer_email='info@example.com',
    description='A* path planning package for AGV autonomous navigation',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'path_planner_node = agv_path_planner.path_planner_node:main',
        ],
    },
)
