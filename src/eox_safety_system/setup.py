from setuptools import find_packages, setup

package_name = 'eox_safety_system'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Frenk Schulten',
    maintainer_email='frenkschulten@eoxtractors.com',
    description='Data extraction and visualization based on Bosch radar, USS, and FLIR ADK thermal camera',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
            'braking_node = eox_safety_system.braking_control_node:main',
            'vehicle_simulator = eox_safety_system.vehicle_simulator:main',
            'video_publisher = eox_safety_system.video_publisher:main',
        ],
    },
)

