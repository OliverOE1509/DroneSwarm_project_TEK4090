from setuptools import setup

package_name = 'drone_swarm_ctf_package'

data_files = [
    ('share/ament_index/resource_index/packages',
     ['resource/' + package_name]),
    ('share/' + package_name + '/launch',
     ['launch/two_drones_webots.launch.py']),
    ('share/' + package_name + '/worlds',
     ['worlds/mavic_2_pro.wbt']),
    ('share/' + package_name + '/resource',
     ['resource/drone1.urdf', 'resource/drone2.urdf']),
    ('share/' + package_name, ['package.xml']),
]

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Oliver Ekeberg',
    maintainer_email='oliveroekeberg@gmail.com',
    description='Two Mavic drones in Webots exchanging ROS 2 state.',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
    'console_scripts': [
        'drone1_driver = drone_swarm_ctf_package.drone1_driver:main',
        'drone2_driver = drone_swarm_ctf_package.drone2_driver:main',
    ],
},
)
