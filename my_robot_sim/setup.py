from setuptools import find_packages, setup
import os

package_name = 'my_robot_sim'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/sim_launch.py']),
        ('share/' + package_name + '/worlds', ['worlds/maze_lidar.sdf']),  # <-- add your world here
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='olli',
    maintainer_email='ollikivi@gmail.com',
    description='simple gazebo simulation',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'obstacle_avoidance = my_robot_sim.obstacle_avoidance:main',
        ],
    },
)
