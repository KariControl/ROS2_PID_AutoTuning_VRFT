from launch import LaunchDescription
from launch_ros.actions import Node
import os

pkg_name = 'vehicle_sim'

def generate_launch_description():
    ld = LaunchDescription()
    node = Node(
        package=pkg_name,
        executable='vehicle_sim',
        name='vehicle_node',
        output='screen',
        parameters=[{
        'time_constant': 1.0,  # 時定数T
        'DC_gain': 0.8,  # DCゲイン
        'diff_time': 0.1,  # サンプリング時間
        'vehicle_speed': 0.0,  # 初期車速
        }]
    )
    ld.add_action(node)
    return ld
