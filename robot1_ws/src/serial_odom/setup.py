from setuptools import find_packages, setup

package_name = 'serial_odom'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=[
    'setuptools',
    'pyserial',
	],

    zip_safe=True,
    maintainer='ksvsd',
    maintainer_email='ksvsd@example.com',  # <-- replace with real email if desired
    description='Reads odometry data from serial and publishes as a ROS 2 Odometry message.',
    license='MIT',  # <-- change if using a different license
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'serial_odom_node = serial_odom.serial_odom_node:main',
            'serial_odom_node_final = serial_odom.serial_odom_node_final:main',
            
            'serial_odom_node3 = serial_odom.serial_odom_node3:main',
            'serial_odom_node4 = serial_odom.serial_odom_node4:main',
            
            
            'dd_serial_node = serial_odom.diff_drive_serial:main',
            'serial_odom_new = serial_odom.serial_odom_new:main',
            'imu_tester = serial_odom.imu_tester:main',
            
            
        ],
    },
)

