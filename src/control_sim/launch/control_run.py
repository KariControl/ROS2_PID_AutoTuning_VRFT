from launch import LaunchDescription
from launch_ros.actions import Node
import os

pkg_name = 'control_sim'

def generate_launch_description():
    ld = LaunchDescription()
    node = Node(
        package=pkg_name,
        executable='control_sim',
        name='control_sim',
        output='screen',
        parameters=[{
        'kp': 2.110582,  # 比例ゲイン デフォルト0.2
        'ki': 0.125190,  # 積分ゲイン　デフォルト0.3
        'kd': 0.05,  # 微分ゲイン
        'dt': 0.01,  # サンプリング時間
        }]
    )

    ld.add_action(node)
    return ld