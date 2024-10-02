import os
from glob import glob
from setuptools import setup

package_name = 'position_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/ament_index/resource_index/packages',
            ['resource/' + 'visualize.rviz']),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*launch.[pxy][yma]*')),
        (os.path.join('share', package_name), glob('resource/*rviz'))
        # (os.path.join('share', package_name), ['scripts/TerminatorScript.sh'])
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Braden',
    maintainer_email='braden@arkelectron.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
                'offboard_control = position_control.offboard_control:main',
                'visualizer = position_control.visualizer:main',
                'position_control = position_control.position_control:main',
                'control = position_control.control:main',
                'processes = position_control.processes:main',
                'position_log = position_control.position_log:main',
                'slave_drone_control = position_control.slave_drone_control:main'
        ],
    },
)
