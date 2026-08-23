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
        # 주행성 지도 평가 도구. 다른 방식(SLAM 등)과 성능을 비교할 때
        # 이 스크립트들을 그대로 써야 숫자가 같은 기준으로 나온다.
        # 특히 eval_connectivity.py 의 "최대 연결덩어리"가 판정 지표다.
        (os.path.join('lib', package_name),
         glob('scripts/*.py')),
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
