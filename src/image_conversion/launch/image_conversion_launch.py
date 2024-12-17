from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='usb_cam',
            executable='usb_cam_node_exe',
            name='usb_cam_node',
            parameters=[{'frame_rate': 30}],
            output='screen',
        ),
        Node(
            package='image_conversion',
            executable='image_conversion_node',
            name='image_conversion_node',
            parameters=[
                {'input_topic': '/usb_cam/image_raw'},
                {'output_topic': '/converted_image'}
            ],
            output='screen',
        )
    ])
