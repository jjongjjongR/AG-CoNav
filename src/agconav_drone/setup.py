from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'agconav_drone'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools', 'pyyaml'],
    zip_safe=True,
    maintainer='yeonju',
    maintainer_email='TODO@example.com',
    description='모듈 A - 드론 2.5D 지도 생성 (경로 재생 노드)',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'drone_path_player = agconav_drone.drone_path_player:main',
            'drone_pose_controller = agconav_drone.drone_pose_controller:main',
        ],
    },
)
