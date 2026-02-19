from setuptools import setup

package_name = 'infa_inference'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='INF-A Team',
    maintainer_email='dev@infa.local',
    description='INF-A defect inference node with OBB detection persistence and event publishing.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'defect_inference_node = infa_inference.defect_inference_node:main',
        ],
    },
)
