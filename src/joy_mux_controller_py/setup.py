from setuptools import find_packages, setup
from glob import glob

package_name = 'joy_mux_controller_py'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='scrb',
    maintainer_email='michael.lythgoe@spaceconcordia.ca',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'joy_mux_controller = joy_mux_controller_py.joy_mux_controller:main',
            'fk_keyboard_controller = joy_mux_controller_py.fk_keyboard_controller:main',
            'fk_moveit_keyboard_controller = joy_mux_controller_py.fk_moveit_keyboard_controller:main',
        ],
    },
)
