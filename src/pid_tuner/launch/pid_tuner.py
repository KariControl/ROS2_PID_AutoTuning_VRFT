from launch import LaunchDescription
from launch_ros.actions import Node
import os

pkg_name = 'pid_tuner'

def generate_launch_description():
    ld = LaunchDescription()
    node = Node(
        package=pkg_name,
        executable='pid_tuner',
        name='tuner_node',
        output='screen',
        parameters=[{
        'time_const': 1.5,  # 参照モデルの時定数T
        'max_data_points': 150,  # データ数
        'diff_time': 0.1,  # サンプリング時間
        'mode_selector': 1,  # 0:PI制御、1:PID制御
        }]
    )

    ld.add_action(node)
    return ld
