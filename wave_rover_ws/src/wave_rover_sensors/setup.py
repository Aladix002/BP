from setuptools import find_packages, setup

package_name = 'wave_rover_sensors'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Wave Rover',
    maintainer_email='robot@waverover.local',
    description='Sensor nodes for Wave Rover.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'imu_node       = wave_rover_sensors.imu_node:main',
            'camera_node    = wave_rover_sensors.camera_node:main',
            'web_server     = wave_rover_sensors.web_server_node:main',
        ],
    },
)
