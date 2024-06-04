from launch import LaunchDescription
from launch_ros.actions import Node
import os

pkg_name1 = 'control_sim'
pkg_name2 = 'vehicle_sim'

def generate_launch_description():
    ld = LaunchDescription()

    config1 = os.path.join(
        pkg_name1,
        'config'
    )    
    config2 = os.path.join(
        pkg_name2,
        'config'
    )

    node1 = Node(
        package=pkg_name1,
        executable='control_sim',
        name='control_sim',
        output='screen',
        parameters=[config1]
    )

    node2 = Node(
        package=pkg_name2,
        executable='vehicle_sim',
        name='vehicle_node',
        output='screen',
        parameters=[config2]
    )

    ld.add_action(node1)
    ld.add_action(node2)

# 複数ノードを追加する場合は，configN,nodeNを作ってld.add_action(nodeN)?

    return ld