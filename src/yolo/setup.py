#! /usr/bin/python3.10
from setuptools import setup

package_name = 'yolo'

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
    maintainer='tumi6',
    maintainer_email='shixin.li@tum.de',
    description='node that use yolov8(ultralytic) to implement object detection',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'yolo_node = yolo.yolo_node:main'
        ],
    },
)
