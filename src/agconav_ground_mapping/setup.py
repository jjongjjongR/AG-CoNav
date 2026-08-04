from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'agconav_ground_mapping'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='brian',
    maintainer_email='brian7025@gmail.com',
    description=(
        'AG-CoNav module D: ground robot (wheel/leg) 2.5D elevation map '
        'accumulation.'
    ),
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ground_elevation_mapper = '
            'agconav_ground_mapping.ground_elevation_mapper:main',
            'ground_elevation_map_saver = '
            'agconav_ground_mapping.ground_elevation_map_saver:main',
        ],
    },
)
