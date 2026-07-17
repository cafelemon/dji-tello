from setuptools import find_packages, setup

package_name = 'tello_mock'

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
    description='UDP Mock Tello with deterministic fault injection.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={'console_scripts': [
        'mock_tello = tello_mock.mock_tello_node:main',
        'flight_fault_harness = tello_mock.flight_fault_harness:main',
    ]},
)
