from setuptools import find_packages, setup

package_name = 'agv_3d_scanner'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/3d_scanner.launch.py']),
        ('share/' + package_name + '/config', ['config/scanner_config.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='agv',
    maintainer_email='agv@todo.todo',
    description='3D scanning, point cloud processing and map generation for AGV',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'scanner_node = agv_3d_scanner.scanner_node:main',
            'point_cloud_manager = agv_3d_scanner.point_cloud_manager:main',
            'map_exporter = agv_3d_scanner.map_exporter:main',
        ],
    },
)
