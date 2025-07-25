from setuptools import find_packages, setup

package_name = 'gps_core'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='gps-cluster-i',
    maintainer_email='gps-cluster-i@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'run_gps1 = gps_core.run_gps:main',
            'gps_revised = gps_core.gps_revised:main',
        ],
    },
)
