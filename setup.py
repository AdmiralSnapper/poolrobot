import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'poolrobot'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        
        # Grabs ANY file ending in .py inside the launch folder
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.launch.py')) + glob(os.path.join('launch', '*launch.py'))),
        
        # Installs your at_map.yaml file
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.yaml'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='compute',
    maintainer_email='compute@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'tag_position_node = poolrobot.tag_position_node:main',
            'cam_localisation_node = poolrobot.cam_localisation_node:main',
            'cal_checker_node = poolrobot.cal_checker_node:main',
            'raw_image_socket_node = poolrobot.raw_image_socket_node:main',
            'calibrator_node = poolrobot.calibrator_node:main',
        ],
    },
)
