import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'thrust_nav_sim'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
        (os.path.join('share', package_name, 'sdf'), glob('sdf/*')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    description='Gazebo-симуляция гусеничного робота и мост ros_gz.',
    entry_points={
        'console_scripts': [
            'spawn_when_world_ready = thrust_nav_sim.spawn_when_world_ready:main',
        ],
    },
)
