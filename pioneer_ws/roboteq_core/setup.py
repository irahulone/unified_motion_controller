from setuptools import find_packages, setup

package_name = 'roboteq_core'
publishers = 'roboteq_core/publishers'
subscribers = 'roboteq_core/subscribers'

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
    maintainer='wallee',
    maintainer_email='wallee@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'roboteq_interface = roboteq_core.main:main'
        ],
    },
)
