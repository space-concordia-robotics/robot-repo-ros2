from setuptools import find_packages, setup

package_name = 'science_payload'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test', 'seabreeze']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Will Free',
    maintainer_email='will.free@spaceconcordia.ca',
    description='Science Payload',
    license='MIT',
    entry_points={
        'console_scripts': [
            'science_collection = science_payload.science_collection:main',
            'spectrometer_service = science_payload.spectrometer_service:main'
        ],
    },
)
