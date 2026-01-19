import os
from glob import glob
from setuptools import setup

package_name = 'aruco_marker_analysis'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
        ('lib/' + package_name, glob(package_name + '/*.py')),
    ],
    install_requires=['setuptools', 'opencv-python', 'numpy'],
    zip_safe=True,
    maintainer='kasynx',
    maintainer_email='cyrilszekiel.aranas@gmail.com',
    description='ArUco marker pose analysis for verticality detection',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'aruco_pose_analyzer = aruco_marker_analysis.aruco_pose_analyzer:main',
        ],
    },
)

