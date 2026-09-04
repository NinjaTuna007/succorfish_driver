import os
from glob import glob

from setuptools import setup

package_name = 'succorfish_driver'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch') + glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools', 'pyserial'],
    extras_require={'test': ['pytest']},
    zip_safe=True,
    maintainer='Shekhar Devm Upadhyay',
    maintainer_email='sdup@kth.se',
    description='Transparent ROS 2 serial bridge that exclusively owns the '
                'Succorfish modem / Teensy serial port and exposes it as RX/TX '
                'text topics, byte-frame topics, plus a generic SendCommand service.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'succorfish_driver_node = succorfish_driver.succorfish_driver_node:main',
        ],
    },
)
