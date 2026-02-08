from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'hamals_serial_bridge'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),

        ('share/' + package_name, ['package.xml']),

        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),

        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='m-gnr',
    maintainer_email='m_gnr@icloud.com',
    description='ROS2 serial bridge between Hamals MCU and ROS',
    license='MIT',
    extras_require={
        'test': ['pytest'],
    },
    entry_points={
        'console_scripts': [
            'serial_node = hamals_serial_bridge.serial_node:main',
        ],
    },
)