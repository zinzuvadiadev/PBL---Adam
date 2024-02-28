from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [launch_file_path]  # Replace with the path to my_cam.launch
            ),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [launch_file_path]  # Replace with the path to hand_sign.launch
            ),
        ),
        DeclareLaunchArgument('publish_gesture_topic', default_value=['/gesture/hand_sign']),
        DeclareLaunchArgument('control_topic', default_value=['/robot_diff_drive_controller/cmd_vel']),
        Node(
            package='ros_hand_gesture_recognition',
            executable='sign_to_controller.py',
            name='sign_to_controller',
            output='screen',
            parameters=[
                {'publish_gesture_topic': [DeclareLaunchArgument('publish_gesture_topic')],
                 'control_topic': [DeclareLaunchArgument('control_topic')]}
            ],
        ),
    ])
