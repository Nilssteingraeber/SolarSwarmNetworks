from setuptools import find_packages, setup

package_name = 'sw_robot'

setup(
    name=package_name,
    version='0.6.3',
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
    description='The main package containing robot nodes for the SolarSwarm project.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'data_sink = sw_robot.data_sink:main'
        ],
    },
)
