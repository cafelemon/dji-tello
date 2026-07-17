from setuptools import find_packages, setup

package_name = 'tello_vision'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Fei Jia',
    maintainer_email='fei.jia@groupe-esigelec.org',
    description='Latest-frame YOLOv5 and ByteTrack vision node for Tello Edge.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'tracker_node = tello_vision.tracker_node:main',
            'offline_video_publisher = tello_vision.offline_video_publisher:main',
        ],
    },
)
