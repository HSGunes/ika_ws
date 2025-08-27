from setuptools import setup

package_name = 'ika_nav2'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/simple_navigation.launch.py']),
        ('share/' + package_name + '/config', ['config/nav2_params.yaml']),
        ('share/' + package_name + '/maps', ['maps/map.yaml', 'maps/map.pgm']),
        ('share/' + package_name + '/behavior_trees', ['behavior_trees/navigate_w_replanning_and_recovery.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='gunes',
    maintainer_email='hikmetselcukgunes@gmail.com',
    description='IKA robot için Nav2 tabanlı otonom navigasyon paketi',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
        ],
    },
)
