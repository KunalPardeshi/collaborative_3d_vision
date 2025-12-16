from setuptools import find_packages, setup

package_name = 'zed_hmi_streamer'

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
		'mjpeg_server = zed_hmi_streamer.mjpeg_server:main',
		'yolo_seg_server = zed_hmi_streamer.yolo_seg_mjpeg_server:main',
		'yolo_pose_server = zed_hmi_streamer.yolo_pose_mjpeg_server:main',
		'pose_hands_server = zed_hmi_streamer.yolo_pose_hands_mjpeg_server:main',
		'combined_hmi = zed_hmi_streamer.combined_hmi_server:main',
        ],
    },
)
