from setuptools import setup
import os
from glob import glob

package_name = 'tello_ros2'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Kullanıcı',
    maintainer_email='davutenees4@yaani.com',
    description='Tello Drone için ROS 2 Humble arayüzü',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'tello_node = tello_ros2.tello_node:main',
        ],
    },
)