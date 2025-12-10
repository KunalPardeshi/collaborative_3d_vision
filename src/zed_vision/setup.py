from setuptools import setup
import os
from glob import glob

package_name = 'zed_vision'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='manipulator',
    maintainer_email='manipulator@todo.todo',
    description='ZED visualization tools',
    license='TODO',
    entry_points={
        'console_scripts': [
            'obj_viz_all = zed_vision.obj_viz_all:main',
            'object_coords = zed_vision.coords_publisher:main',
        ],
    },
)
