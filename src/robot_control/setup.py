from setuptools import setup


package_name = 'robot_control'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/avoidance.yaml']),
        ('share/' + package_name + '/launch', ['launch/avoidance.launch.py']),
    ],
    install_requires=['setuptools'],
    maintainer='Aman Nawabi',
    maintainer_email='aman.nawabi@amannawabi.com',
    description='ROS 2 control and obstacle avoidance for the four-wheel robot',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'scan_listener = robot_control.scan_listener:main',
            'avoidance_controller = robot_control.avoidance_controller:main',
            'vl53l0x_range = robot_control.vl53l0x_range:main',
            'motor_bonnet = robot_control.motor_bonnet:main',
        ],
    },
)
