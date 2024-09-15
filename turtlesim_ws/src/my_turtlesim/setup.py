from setuptools import find_packages, setup

package_name = 'my_turtlesim'

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
    maintainer='Duke Robotics Club',
    maintainer_email='hello@duke-robotics.com',
    description='Code for controlling turtle in turtlesim.',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'move_turtle = my_turtlesim.move_turtle:main',
        ],
    },
)
