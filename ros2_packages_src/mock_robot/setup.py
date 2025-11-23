from setuptools import find_packages, setup

package_name = 'mock_robot'

# v0.14.0 with new test services from custom_interfaces v0.4.0
setup(
    name=package_name,
    version='0.14.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Vincent Welsch',
    maintainer_email='vincent.welsch@stud.hs-bochum.de',
    description='This package implements a mock robot which publishes mock data about itself.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mock_data = mock_robot.mock_data:main',
            'fibonacci = mock_robot.fibonacci:main',
            'get_service_server_mac = mock_robot.get_service_server_mac:main',
            'int_avg = mock_robot.int_avg:main',
            'int_rng = mock_robot.int_rng:main',
            'str_to_lower_upper = mock_robot.str_to_lower_upper:main',
            
        ],
    },
)
