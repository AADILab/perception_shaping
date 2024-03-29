import os
from glob import glob
from setuptools import setup

package_name = 'perception_shaping'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include all launch files.
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*')))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='everardo.a.gonzalez@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'driver = perception_shaping.drive_to_poi:main',
            'scan_cleaner = perception_shaping.clean_scan:main',
            'gazebo_pose_publisher = perception_shaping.gazebo_pose_publisher:main',
            'supervisor = perception_shaping.supervisor_node:main'
        ],
    },
)
