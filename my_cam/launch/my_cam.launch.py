from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('camera_name', default_value="/dev/video0"),
        Node(
            package='my_cam',
            executable='Cam',
            name='my_webcam',
            output='screen',
            parameters=[{'camera_name': '/dev/video0'}]
        ),
    ])
