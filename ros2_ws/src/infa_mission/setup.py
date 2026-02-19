from setuptools import setup

package_name = 'infa_mission'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/infa_mission.launch.py']),
        ('share/' + package_name + '/scripts', ['scripts/validate_mission_sim.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='INF-A Team',
    maintainer_email='dev@infa.local',
    description='Mission control, standoff lock, and sweep planning for INF-A.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mission_node = infa_mission.mission_node:main',
        ],
    },
)
