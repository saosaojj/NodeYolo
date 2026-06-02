import os
from glob import glob
from setuptools import setup

package_name = 'agv_web_frontend'

# 收集前端静态文件
def collect_data_files():
    """收集前端静态文件用于安装"""
    data_files = [
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ]

    # 安装 index.html
    if os.path.exists('index.html'):
        data_files.append(
            (os.path.join('share', package_name), ['index.html'])
        )

    # 安装 CSS 文件
    css_files = glob('css/*.css')
    if css_files:
        data_files.append(
            (os.path.join('share', package_name, 'css'), css_files)
        )

    # 安装 JS 文件
    js_files = glob('js/*.js')
    if js_files:
        data_files.append(
            (os.path.join('share', package_name, 'js'), js_files)
        )

    # 安装 assets 文件（如果存在）
    assets_files = glob('assets/*')
    if assets_files:
        data_files.append(
            (os.path.join('share', package_name, 'assets'), assets_files)
        )

    return data_files


setup(
    name=package_name,
    version='1.0.0',
    packages=[],
    data_files=collect_data_files(),
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
