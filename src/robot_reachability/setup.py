from setuptools import find_packages, setup

package_name = 'robot_reachability'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/reachability_scan.launch.py']),
        ('share/' + package_name + '/config', ['config/reachability_scan.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='minhquang',
    maintainer_email='minhquang@example.com',
    description='Reachability map scanner for ROS 2 MoveIt 2 robot arms.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'reachability_scan_node = robot_reachability.reachability_scan_node:main',
        ],
    },
)
