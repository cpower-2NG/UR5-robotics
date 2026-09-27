from setuptools import find_packages, setup

package_name = 'grasp_planner'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/planner.launch.py']),
        ('share/' + package_name + '/config', ['config/grasp_planner.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@example.com',
    description='Grasp planning with MoveIt2 IK',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'planner_node = grasp_planner.planner_node:main',
        ],
    },
)
