from setuptools import find_packages, setup

package_name = 'tello_system_monitor'

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
    description='Host and Jetson diagnostics publisher for Tello Edge.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'system_monitor = tello_system_monitor.system_monitor_node:main',
        ],
    },
)
