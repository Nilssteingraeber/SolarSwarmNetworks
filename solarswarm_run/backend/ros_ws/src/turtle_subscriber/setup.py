from setuptools import find_packages, setup

package_name = 'turtle_subscriber'

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
    maintainer='root',
    maintainer_email='christopher.bluemer@stud.hs-bochum.de',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
	   'subscriber = turtle_subscriber.turtle_pose_subscriber:main',
	   'api_subscriber = turtle_subscriber.api_turtle_pose_subscriber:main'
        ],
    },
)
