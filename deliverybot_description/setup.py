from setuptools import setup
from os import path
from setuptools import find_packages
from glob import glob
import os

package_name = 'deliverybot_description'
# robot_folder = 'robot'
# models_folder = 'models'
# launch_folder = 'launch'
# rviz_folder = 'rviz'
# config_folder = 'config'

data_files = [
    ('share/ament_index/resource_index/packages',
     ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),
]

def package_files(data_files, directory_list):
    paths_dict = {}
    for directory in directory_list:
        for (path, directories, filenames) in os.walk(directory):
            for filename in filenames:
                file_path = os.path.join(path, filename)
                install_path = os.path.join('share', package_name, path)
                if install_path in paths_dict.keys():
                    paths_dict[install_path].append(file_path)
                else:
                    paths_dict[install_path] = [file_path]
    for key in paths_dict.keys():
        data_files.append((key, paths_dict[key]))
    return data_files

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=package_files(data_files, ['models/','launch/','worlds/', 'rviz/', 'xacro/']),
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
