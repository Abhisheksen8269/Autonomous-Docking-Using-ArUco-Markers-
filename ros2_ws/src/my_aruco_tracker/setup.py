import os
from glob import glob
from setuptools import setup

package_name = 'my_aruco_tracker'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),

        # IMPORTANT: install launch files
        # IMPORTANT: install launch files
        ('share/' + package_name + '/launch', glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='abhishek',
    maintainer_email='',
    description='Aruco tracker package',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'aruco_tracker = my_aruco_tracker.aruco_tracker:main',
            'detect_node = my_aruco_tracker.detect_node:main',
            'generate_aruco = my_aruco_tracker.generate_aruco:main',
        ],
    },
)
