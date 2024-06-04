from launch import LaunchDescription
from launch_ros.actions import Node
import os

pkg_name = 'control_sim'

def generate_launch_description():
    ld = LaunchDescription()

    config1 = os.path.join(
        pkg_name,
        'config'
    )    

    node1 = Node(
        package=pkg_name,
        executable='control_sim',
        name='control_sim',
        output='screen',
        parameters=[config1]
    )

    ld.add_action(node1)
    return ld