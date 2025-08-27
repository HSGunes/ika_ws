from setuptools import setup
import os
from glob import glob

package_name = 'ika_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.yaml'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@todo.todo',
    description='Controller package for ika 6-wheel rover',
    license='TODO',
    entry_points={
        'console_scripts': [
            'ika_controller = ika_controller.ika_controller:main',
            'ika_bridge = ika_controller.ika_bridge:main',
            'rover_teleop = ika_controller.rover_teleop:main',
            'lidar_centering = ika_controller.lidar_centering:main',
        ],
    },
) 