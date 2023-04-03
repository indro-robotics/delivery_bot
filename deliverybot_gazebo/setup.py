from setuptools import setup
from os import path
from setuptools import find_packages
from glob import glob

package_name = 'deliverybot_gazebo'
launch_folder = 'launch'
worlds_folder = 'worlds'
setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (path.join('share', package_name, launch_folder), glob('launch/*', recursive=True)),
        (path.join('share', package_name, worlds_folder), glob('worlds/*', recursive=True))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='liamd',
    maintainer_email='liamd@indrorobotics.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'deliverybot_gazebo = deliverybot_gazebo.deliverybot_gazebo:main'
        ],
    },
)
