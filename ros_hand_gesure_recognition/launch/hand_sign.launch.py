from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('keypoint_classifier_label', default_value=[
            '$(find ros_hand_gesture_recognition)/src/model/keypoint_classifier/keypoint_classifier_label.csv']),
        DeclareLaunchArgument('keypoint_classifier_model', default_value=[
            '$(find ros_hand_gesture_recognition)/src/model/keypoint_classifier/keypoint_classifier.tflite']),
        DeclareLaunchArgument('subscribe_image_topic', default_value=['/image_raw']),
        DeclareLaunchArgument('publish_gesture_topic', default_value=['/gesture/hand_sign']),
        DeclareLaunchArgument('show_image', default_value=[True]),

        Node(
            package='ros_hand_gesture_recognition',
            executable='gesture',
            name='hand_sign_recognition',
            output='screen',
            parameters=[
                {'keypoint_classifier_label': [DeclareLaunchArgument('keypoint_classifier_label')],
                 'keypoint_classifier_model': [DeclareLaunchArgument('keypoint_classifier_model')],
                 'subscribe_image_topic': [DeclareLaunchArgument('subscribe_image_topic')],
                 'publish_gesture_topic': [DeclareLaunchArgument('publish_gesture_topic')],
                 'show_image': [DeclareLaunchArgument('show_image')]}
            ],
        ),
    ])
