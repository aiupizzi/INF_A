from setuptools import find_packages, setup

package_name = 'infa_sync'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', [f'resource/{package_name}']),
        (f'share/{package_name}', ['package.xml']),
        (f'share/{package_name}/config', ['config/example_sync.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='INF-A Team',
    maintainer_email='dev@infa.local',
    description='ROS 2 node that uploads mission imagery to WebODM and monitors reconstruction.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'sync_node = infa_sync.sync_node:main',
        ],
    },
)
