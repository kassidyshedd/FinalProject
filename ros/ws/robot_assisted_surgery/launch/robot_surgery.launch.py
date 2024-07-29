from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='robot_assisted_surgery',
            executable='camera_node',
            name='camera_node',
            output='screen'
        )
    ])