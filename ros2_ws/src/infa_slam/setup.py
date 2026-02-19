from setuptools import setup

package_name = 'infa_slam'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/infa_slam.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='INF-A Team',
    maintainer_email='dev@infa.local',
    description='INF-A SLAM/VIO integration and local structure estimation publishers.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'slam_bridge_node = infa_slam.slam_bridge_node:main',
        ],
    },
)
