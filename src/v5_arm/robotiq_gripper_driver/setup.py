from setuptools import find_packages, setup
import os
import glob

package_name = 'robotiq_gripper_driver'

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
    maintainer='nikunj',
    maintainer_email='carsonstark@ymail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'gripper_driver = robotiq_gripper_driver.gripper_ros_controller:main'
        ],
    },
)
