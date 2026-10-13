import os
from glob import glob

from setuptools import setup

package_name = 'hsr_omniverse_ros2'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
         glob('launch/*.launch.py') + glob('launch/*.launch.xml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Ryohei Kobayashi',
    maintainer_email='kobayashi.ryohei621@mail.kyutech.jp',
    description='Isaac Sim 上の HSR を ROS 2 の HSR スタックに繋ぐ launch と中継ノード',
    license='BSD-3-Clause',
    entry_points={
        'console_scripts': [
            'rgbd_republisher = hsr_omniverse_ros2.rgbd_republisher:main',
            'reset_world_matcher_helper = hsr_omniverse_ros2.reset_world_matcher_helper:main',
        ],
    },
)
