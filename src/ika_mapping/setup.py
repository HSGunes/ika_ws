from setuptools import setup
import os
from glob import glob

package_name = 'ika_mapping'

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
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.pgm'))),
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.yaml'))),
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.lua'))),
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.pbstream'))),
        (os.path.join('share', package_name, 'maps'), glob(os.path.join('maps', '*.pgm'))),
        (os.path.join('share', package_name, 'maps'), glob(os.path.join('maps', '*.yaml'))),
        (os.path.join('share', package_name, 'worlds'), glob(os.path.join('worlds', '*.world'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='gunes',
    maintainer_email='gunes@todo.todo',
    description='SLAM and Navigation package for IKA robot',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
        ],
    },
)
