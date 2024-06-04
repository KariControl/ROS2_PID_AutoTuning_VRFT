from launch import LaunchDescription
from launch_ros.actions import Node
import os

pkg_name2 = 'vehicle_sim'

def generate_launch_description():
    ld = LaunchDescription()

    config2 = os.path.join(
        pkg_name2,
        'config'
    )

    node2 = Node(
        package=pkg_name2,
        executable='vehicle_sim',
        name='vehicle_node',
        output='screen',
        parameters=[config2]
    )

    ld.add_action(node2)
    return ld