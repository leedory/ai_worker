from glob import glob

from setuptools import find_packages, setup

package_name = 'ffw_april_head_calibration'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    package_data={
        package_name: ['apriltag_detect_worker.py'],
    },
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools', 'PyYAML'],
    zip_safe=True,
    maintainer='Pyo',
    maintainer_email='pyo@robotis.com',
    description='AprilTag head calibration / tracking for FFW / ai_worker.',
    license='Apache 2.0',
    entry_points={
        'console_scripts': [
            'april_head_calibration = ffw_april_head_calibration.april_head_tracker_node:main',
            'ffw_apply_head_homing_offset_from_delta = '
            'ffw_april_head_calibration.apply_head_homing_offset_from_delta:main',
        ],
    },
)
