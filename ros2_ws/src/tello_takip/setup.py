from setuptools import setup
import os
from glob import glob

package_name = 'tello_takip'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # YOLO model dosyasını ekleyin
        (os.path.join('share', package_name, 'models'), glob('models/*.pt')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Kullanıcı Adı',
    maintainer_email='kullanici@email.com',
    description='Tello drone ile nesne tespit ve takip uygulaması',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'tello_nesne_tespit = tello_takip.tello_nesne_tespit:main',
        ],
    },
)