from setuptools import find_packages, setup

package_name = 'wave_rover_control'

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
    description='Control nodes for Wave Rover.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'motor_controller = wave_rover_control.motor_controller_node:main',
            'imu_stabilizer   = wave_rover_control.imu_stabilizer_node:main',
            'mode_manager     = wave_rover_control.mode_manager_node:main',
            'watchdog         = wave_rover_control.watchdog_node:main',
            'shutdown_node    = wave_rover_control.shutdown_node:main',
            'node_manager     = wave_rover_control.node_manager_node:main',
        ],
    },
)
