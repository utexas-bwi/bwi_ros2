from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'bwi_demo'

def get_all_files(directory):
    files = []
    for filepath in glob(os.path.join(directory, '**/*'), recursive=True):
        if os.path.isfile(filepath):  # Ensure it's a file
            files.append(filepath)
    return files

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'data'), glob('data/*.yaml'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='carson',
    maintainer_email='carsonstark@ymail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'arm_test = bwi_demo.arm_test:main',
            'demo = bwi_demo.demo:main',
        ],
    },
)
