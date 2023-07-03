from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='lanturn_vision',
            namespace='lanturn_vision',
            executable='img_pub',
            name='retina_node'
            ),
        Node(
            package='lanturn_vision',
            namespace='lanturn_vision',
            executable='img_sub',
            name='vision_node'
            )
    ])
