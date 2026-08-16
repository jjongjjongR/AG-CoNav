from setuptools import setup

package_name = 'agconav_navigation'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='lsb',
    maintainer_email='lsb@todo.todo',
    description='Navigation module for AG-CoNav (Module C)',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ground_segmentation_node = agconav_navigation.ground_segmentation_node:main',
            'navigation_complete_node = agconav_navigation.navigation_complete_node:main',
            'mock_publisher = agconav_navigation.mock_publisher:main',
            'cmd_vel_to_control_input = agconav_navigation.cmd_vel_to_control_input:main'
        ],
    },
)
