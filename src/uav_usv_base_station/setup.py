from glob import glob
import os

from setuptools import find_packages, setup


package_name = 'uav_usv_base_station'


setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
         glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'),
         glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='UAV-USV Team',
    maintainer_email='team@example.com',
    description='Read-only fleet situation cache and event service.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'base_station_service = '
            'uav_usv_base_station.base_station_service_node:main',
        ],
    },
)
