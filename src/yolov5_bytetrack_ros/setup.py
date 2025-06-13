from setuptools import find_packages, setup

package_name = 'yolov5_bytetrack_ros'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(include=[
        'yolov5_bytetrack_ros',
        'yolov5_bytetrack_ros.*',
        'yolox',
        'yolox.tracker'
    ]),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jf',
    maintainer_email='jf@todo.todo',
    description='YOLOv5 + ByteTrack ROS2 integration for Tello drone tracking',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'tracker_node = yolov5_bytetrack_ros.tracker_node:main',
            'tello_image_publisher = yolov5_bytetrack_ros.tello_image_publisher:main',
            'tello_video_subscriber = yolov5_bytetrack_ros.tello_video_subscriber:main',
            'tello_video_bridge = yolov5_bytetrack_ros.tello_cmd_vel_bridge:main'
        ],
    },
)

