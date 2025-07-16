
import os
from glob import glob
from setuptools import setup

package_name = 'remote_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='yuchen',
    maintainer_email='yuchen.liu@campus.tu-berlin.de',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'rc_publisher = remote_control.rc_publisher:main',
            'rc_to_ackermann = remote_control.rc_to_ackermann:main',
            'rc_to_vehicle_command = remote_control.rc_to_vehicle_command:main',
            'simple_vesc_interface = remote_control.simple_vesc_interface:main',
        ],
    },
)
