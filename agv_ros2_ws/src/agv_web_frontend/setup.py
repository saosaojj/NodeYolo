import os
from glob import glob
from setuptools import setup

package_name = 'agv_web_frontend'

setup(
    name=package_name,
    version='1.0.0',
    packages=[],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # 安装前端静态文件
        (os.path.join('share', package_name, 'css'), glob('css/*.css')),
        (os.path.join('share', package_name, 'js'), glob('js/*.js')),
        (os.path.join('share', package_name, 'assets'), glob('assets/*')),
        (os.path.join('share', package_name), ['index.html']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='AGV Team',
    maintainer_email='agv@example.com',
    description='AGV Web Frontend Static Files',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
