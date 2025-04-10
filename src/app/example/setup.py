from setuptools import setup
import os
from glob import glob

package_name = 'example'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@todo.todo',
    description='ROS2 node example to move a vehicle in the simultion',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'example_node = example.example_node:main',
        ],
    },
) 