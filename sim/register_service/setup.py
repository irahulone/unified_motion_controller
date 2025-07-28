from setuptools import setup
import os
from glob import glob

package_name = 'register_service'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='Service example in ROS2',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'register_service = register_service.robot_register:main',
            'register_service_req = register_service.robot_register_request:main',
            'remove_service_req = register_service.robot_remove_request:main',  
        ],
    },
)
