from setuptools import find_packages, setup

package_name = 'object_recognition'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/keymap.yaml']),
        ('share/' + package_name + '/launch', ['launch/keymap._launch.py']),
    ],
    install_requires=['setuptools', 'PyYAML'],
    zip_safe=True,
    maintainer='liliana',
    maintainer_email='lilimariajf@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'keymap_node = object_recognition.keymap_node:main',
        ],
    },
)