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
        # 스캔 경로 생성기. 간격을 바꿔 새 경로를 뽑을 때 쓴다
        # (기본 경로 config/scan_path_fullmap.yaml 도 이걸로 만들었다).
        (os.path.join('lib', package_name), glob('scripts/*.py')),
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
            'drone_velocity_follower = agconav_drone.drone_velocity_follower:main',
            # 아래 둘은 SetEntityPose 순간이동 방식(폐기)의 노드다. 파이프라인은
            # 더 이상 쓰지 않지만, 순간이동과의 비교 실험을 다시 돌릴 수 있게
            # 진입점은 남겨둔다.
            'drone_path_player = agconav_drone.drone_path_player:main',
            'drone_pose_controller = agconav_drone.drone_pose_controller:main',
            'drone_elevation_mapper = agconav_drone.drone_elevation_mapper:main',
            'elevation_map_saver = agconav_drone.elevation_map_saver:main',
        ],
    },
)
