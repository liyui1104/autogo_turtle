from launch_ros.actions import Node

from launch import LaunchDescription

def generate_launch_description():
    turtlesim_node = Node(
        package='turtlesim',
        executable='turtlesim_node',
        name='turtlesim_node',
        parameters=[{
            'background_r': 255,
            'background_g': 255,
            'background_b': 255,
         }]
    )
    return LaunchDescription([
        turtlesim_node
    ])