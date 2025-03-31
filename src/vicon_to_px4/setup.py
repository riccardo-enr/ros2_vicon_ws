import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'vicon_to_px4'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include launch files
        (os.path.join('share', package_name, 'launch'),
         glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='riccardo',
    maintainer_email='riccardo.enrico97@gmail.com',
    description='VICON to PX4 odometry conversion',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'vicon_to_px4 = vicon_to_px4.vicon_to_px4:main',
        ],
    },
)
