from setuptools import setup

package_name = 'usb_camera_node'

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
    maintainer='Senin İsmin',
    maintainer_email='sen@example.com',
    description='USB kamera görüntüsünü yayınlayan ROS 2 düğümü',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'usb_camera_publisher = usb_camera_node.usb_camera_publisher:main',
        ],
    },
)
