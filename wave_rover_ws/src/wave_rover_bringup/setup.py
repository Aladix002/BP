import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'wave_rover_bringup'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'),   glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'),   glob('launch/*.py')),
        (os.path.join('share', package_name, 'scripts'),  glob('scripts/*')),
        (os.path.join('share', package_name, 'web'),      glob('web/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Wave Rover',
    maintainer_email='robot@waverover.local',
    description='Top-level bringup for Wave Rover robot.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mode_switch = wave_rover_bringup.mode_switch_entry:main',
        ],
    },
)
