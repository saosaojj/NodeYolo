from setuptools import find_packages, setup

package_name = 'agv_vision'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/vision.launch.py', 'launch/training.launch.py']),
        ('share/' + package_name + '/config', ['config/vision_config.yaml']),
        ('share/' + package_name + '/models', []),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='agv',
    maintainer_email='agv@todo.todo',
    description='YOLO object detection with local model inference and self-training capabilities for AGV',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'yolo_detector_node = agv_vision.yolo_detector_node:main',
            'yolo_trainer_node = agv_vision.yolo_trainer_node:main',
            'camera_node = agv_vision.camera_node:main',
        ],
    },
)
