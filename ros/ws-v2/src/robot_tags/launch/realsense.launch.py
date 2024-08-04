import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    apriltag_ros_share = get_package_share_directory('robot_tags')

    return LaunchDescription([
        DeclareLaunchArgument(
            'image_rect',
            default_value='/camera/image',
            description='Topic for the rectified image'
        ),
        DeclareLaunchArgument(
            'camera_info',
            default_value='/camera/camera_info',
            description='Topic for the camera info'
        ),
        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(apriltag_ros_share, 'tags.yaml'),
            description='Path to the parameter file'
        ),

        Node(
            package='apriltag_ros',
            executable='apriltag_node',
            name='apriltag_node',
            output='screen',
            remappings=[
                ('image_rect', LaunchConfiguration('image_rect')),
                ('camera_info', LaunchConfiguration('camera_info')),
            ],
            parameters=[LaunchConfiguration('params_file')],
        )
    ])
