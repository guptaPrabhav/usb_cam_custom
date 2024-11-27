from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='usb_cam',
            executable='usb_cam_node_exe',
            name='cam'
        ),
        Node(
            package='usb_cam_custom',
            executable='image_mode_action_server',
            name='server'
        ),
    ])