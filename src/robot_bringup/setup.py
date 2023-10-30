from setuptools import setup
from glob import glob
import os

package_name = 'robot_bringup'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join("share",package_name),glob("launch/*.launch.*")),
        (os.path.join("share",package_name,"params"),glob("params/*.yaml"))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='luismilczarek',
    maintainer_email='luis.milczarek@hotmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
