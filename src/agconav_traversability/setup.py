import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'agconav_traversability'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
         glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'),
         glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='lee',
    maintainer_email='lee@example.com',
    description='AG-CoNav 모듈 F: 드론 2.5D 지도 → wheel/leg 주행 가능 맵 분리.',
    license='TODO',
    entry_points={
        'console_scripts': [
            'terrain_feature_calculator ='
            ' agconav_traversability.terrain_feature_calculator:main',
            'traversability_verdictor ='
            ' agconav_traversability.traversability_verdictor:main',
            'map_save_coordinator ='
            ' agconav_traversability.map_save_coordinator:main',
        ],
    },
)
