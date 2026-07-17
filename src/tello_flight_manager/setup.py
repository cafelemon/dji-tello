from glob import glob
from setuptools import find_packages, setup

package_name = 'tello_flight_manager'

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
    description='Safety flight state machine and command arbitration for Tello Edge.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'flight_manager = tello_flight_manager.flight_manager_node:main',
        ],
    },
)
