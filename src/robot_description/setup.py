import os
from glob import glob
from setuptools import setup

package_name = 'robot_description'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    # package_data={
    #     package_name:[
    #         "params/*.yaml"
    #     ]
    # },
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # (os.path.join('share', package_name), glob("launch/*.launch.*")),
        (os.path.join('share', package_name, "config"), glob("config/*.yaml")),
        (os.path.join('share', package_name, "urdf"), glob("urdf/*.urdf")),
        (os.path.join('share', package_name, "models"), glob("models/*.sdf")),
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
