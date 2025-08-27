from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'model_launcher'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'models'), glob('models/*')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*')),

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='hilal',
    maintainer_email='hilallates@example.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'object_detector_node_exec = model_launcher.object_detector_node:main',
        ],
    },
)
