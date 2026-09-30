from setuptools import find_packages, setup

package_name = 'ros2endo'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name]
        ),
        (
            'share/' + package_name,
            ['package.xml']
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Henry B. Guerrero',
    maintainer_email='hbguerrero@udistrital.edu.co',
    description='ROS2ENDO wall-following robot nodes',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'esp32_bridge = ros2endo.esp32_bridge:main',
            'lidar_node = ros2endo.lidar_node:main',
            'wall_follower = ros2endo.wall_follower:main',
            'wall_follower_estable = ros2endo.wall_follower_estable:main',
            'experiment_logger = ros2endo.experiment_logger:main',
        ],
    },
)
