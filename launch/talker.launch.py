from  launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='cpp_pubsub',
            node_namespace='namespace1',
            node_executable='talker',
            node_name='talker'
        )
    ])