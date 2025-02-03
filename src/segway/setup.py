from setuptools import find_packages, setup
import os
from glob import  glob

package_name = 'segway'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*')))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='nikunj',
    maintainer_email='ayon.s.das@hotmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'segway_hardware_interface = segway.segway_hardware_interface:main',
            'segway_controller = segway.segway_controller:main',
            'segway_faultlog_parser = segway.faultlog_parser:main',
        ],
    },
)
