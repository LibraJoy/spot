from setuptools import setup
import os
from glob import glob

package_name = 'spot_ros2'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='cerlab',
    maintainer_email='jiamingh@andrew.cmu.edu',
    description='Spot robot ROS2 controller',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'spot_base = spot_ros2.spot_base:main',
            'spot_teleop_keyboard = spot_ros2.spot_teleop_keyboard:main',
            'spot_pano_publisher = spot_ros2.test_image_client_ros2_pub:main',
        ],
    },
)

