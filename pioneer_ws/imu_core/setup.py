from setuptools import setup
import os
import glob 
package_name = 'imu_core'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share',package_name,'launch'),glob.glob('launch/*.launch.py')),
        (os.path.join('share',package_name,'config'),glob.glob('config/*.yaml'))

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='pioneer3',
    maintainer_email='pioneer3@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'run_imu = imu_core.imu_node:main',
        ],
    },
)
