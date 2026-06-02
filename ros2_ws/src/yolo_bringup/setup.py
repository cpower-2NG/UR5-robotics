from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'yolo_bringup'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch',
            glob('launch/*.launch.py')),
        ('share/' + package_name + '/config',
            glob('config/*.yaml')),
        ('share/' + package_name + '/worlds',
            glob('worlds/*.sdf')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@example.com',
    description='Top-level bringup for YOLO grasping pipeline',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'test_image_publisher = yolo_bringup.test_image_publisher:main',
        ],
    },
)
