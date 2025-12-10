from setuptools import find_packages, setup

package_name = 'svo_yolo_vision'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='manipulator',
    maintainer_email='manipulator@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
	'yolo_detector = svo_yolo_vision.yolo_detector:main',
        'hmi_server = svo_yolo_vision.hmi_server:main',
        ],
    },
)
