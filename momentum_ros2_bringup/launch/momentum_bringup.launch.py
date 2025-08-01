from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
           package='momentum_ros2_driver',
           executable='momentum_sensor_node',
           name='momentum_sensor_node',
           output='screen' 
        )
    ])