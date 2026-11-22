import os
from glob import glob
from setuptools import setup

package_name = 'robot_description'

setup(
    name=package_name,
    version='0.0.1',
    # Explicitly list the Python package. Other directories are ROS resources,
    # not Python packages; auto-discovery can fail on flat-layout projects.
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.xacro') + glob('urdf/*.urdf')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml') + glob('config/*.yml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Aman Nawabi',
    maintainer_email='aman.nawabi@amannawabi.com',
    description='Four-wheel robot description and visualization',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'odom_tf_broadcaster = robot_description.odom_tf_broadcaster:main',
        ],
    },
)
