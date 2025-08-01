from setuptools import setup

package_name = 'momentum_ros2_bringup'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ("share/" + package_name + "/launch", ["launch/momentum_bringup.launch.py"]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='scalpelspace',
    maintainer_email='info@scalpelspace.com',
    description='Bringup package to launch momentum_ros2_driver',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [],
    },
)
