from setuptools import setup
from os import path
from setuptools import find_packages
from glob import glob

package_name = 'deliverybot_description'
robot_folder = 'robot'
meshes_folder = 'meshes'
launch_folder = 'launch'
rviz_folder = 'rviz'
config_folder = 'config'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        #Adding necessary data files for package folders
        (path.join('share', package_name, robot_folder), glob('robot/*', recursive=True)),
        (path.join('share', package_name, meshes_folder), glob('meshes/*', recursive=True)),
        (path.join('share', package_name, rviz_folder), glob('rviz/*', recursive=True)),
        (path.join('share', package_name, launch_folder), glob('launch/*', recursive=True)),
        (path.join('share', package_name, config_folder), glob('config/*', recursive=True))
    ],
    install_requires=['setuptools', 'launch', 'launch_ros'],
    zip_safe=True,
    maintainer='liamd',
    maintainer_email='liamd@indrorobotics.com',
    description='ROS2 Description Package for DeliveryBot Simulation',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'deliverybot_description = deliverybot_description.deliverybot_description:main',
            'spawn_deliverybot = deliverybot_description.spawn_deliverybot:main',
        ],
    },
)
