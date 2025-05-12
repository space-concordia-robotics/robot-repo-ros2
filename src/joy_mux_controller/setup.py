from setuptools import find_packages, setup

package_name = 'joy_mux_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(),
    data_files=[
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'rclpy', 'sensor_msgs', 'geometry_msgs'],
    zip_safe=True,
    entry_points={
        'console_scripts': [
            'joy_mux_controller = joy_mux_controller.joy_mux_controller:main',
        ],
    },
)
