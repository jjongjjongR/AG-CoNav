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
            'ground_pointcloud_collector = '
            'agconav_ground_mapping.ground_pointcloud_collector:main',
            'ground_lidar_tf_transformer = '
            'agconav_ground_mapping.ground_lidar_tf_transformer:main',
            'ground_elevation_mapper = '
            'agconav_ground_mapping.ground_elevation_mapper:main',
        ],
    },
)
