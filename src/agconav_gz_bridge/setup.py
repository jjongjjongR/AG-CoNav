import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'agconav_gz_bridge'

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
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='lee',
    maintainer_email='lee@example.com',
    description='AG-CoNav Gazebo↔ROS2 센서 브리지 (계약 토픽 이름 정합).',
    license='MIT',
    entry_points={
        'console_scripts': [
            'tf_prefix_relay = agconav_gz_bridge.tf_prefix_relay:main',
        ],
    },
)
