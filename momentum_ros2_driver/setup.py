from setuptools import setup

package_name = 'momentum_ros2_driver'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'momentum_sdk'],
    zip_safe=True,
    maintainer='scalpelspace',
    maintainer_email='info@scalpelspace.com',
    description='ROS2 driver for Momentum board',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'momentum_sensor_node = momentum_ros2_driver.momentum_sensor_node:main'
        ],
    },
)
