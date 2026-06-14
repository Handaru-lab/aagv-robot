import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'aagv_perception'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='awmc',
    maintainer_email='you@example.com',
    description='Colored ball detection -> 3D -> base_link via tf2.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'ball_detector = aagv_perception.ball_detector:main',
        ],
    },
)
