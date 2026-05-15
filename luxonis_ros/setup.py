import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'luxonis_ros'
_nn_basename = 'best_190_Epoch.rvc2.tar.xz'
_repo_nn = os.path.join(os.path.dirname(__file__), '..', 'luxonis_scripts', _nn_basename)

data_files = [
    ('share/ament_index/resource_index/packages',
        ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),
    (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
]
if os.path.isfile(_repo_nn):
    data_files.append(
        (os.path.join('share', package_name, 'models'), [_repo_nn]),
    )

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='scrb',
    maintainer_email='michael.lythgoe@spaceconcordia.ca',
    description='ROS 2 bridge for Luxonis DepthAI spatial pipelines.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'spatial_camera_node = luxonis_ros.spatial_camera_node:main',
            'ffc_camera_node = luxonis_ros.ffc_camera_node:main',
        ],
    },
)
