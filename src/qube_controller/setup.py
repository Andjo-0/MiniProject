from setuptools import find_packages, setup

package_name = 'qube_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=[
        'setuptools',
        'rclpy',
        'std_msgs',
        'sensor_msgs',
    ],
    zip_safe=True,
    maintainer='andreas-jortveit',
    maintainer_email='andreas-jortveit@hotmail.com',
    description='PID controller node for the Qube servo using ROS 2.',
    license='MIT',  #
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pid_controller = qube_controller.PID_controller:main',
        ],
    },
)
