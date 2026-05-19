from setuptools import find_packages, setup

package_name = 'perception_3d'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/perception.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@example.com',
    description='2D-to-3D projection node',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'projector_node = perception_3d.projector_node:main',
        ],
    },
)
