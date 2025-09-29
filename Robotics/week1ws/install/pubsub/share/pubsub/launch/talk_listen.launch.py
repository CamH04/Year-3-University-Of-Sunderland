from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='pubsub',
            executable='pubnode',
            name='talker'
        ),
        Node(
            package='pubsub',
            executable='subnode',
            name='listener'
        ),
    ])

