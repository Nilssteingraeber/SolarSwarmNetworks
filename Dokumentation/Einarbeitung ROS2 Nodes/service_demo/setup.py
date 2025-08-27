from setuptools import find_packages, setup

package_name = 'service_demo'

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
    maintainer='ubuntuROS2',
    maintainer_email='ubuntuROS2@todo.todo',
    description='Einfache Umsetzung eines Services mit benutzerdefiniertem Dateninterface',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'echo_service = service_demo.simple_service:main',
            'echo_client = service_demo.simple_client:main',
        ],
    },
)
