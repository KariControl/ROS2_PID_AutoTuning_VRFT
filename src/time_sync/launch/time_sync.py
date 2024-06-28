from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='time_sync',  # パッケージ名
            executable='time_sync',  # 実行可能ファイル名
            name='sync_node',  # ノード名
            output='screen',  # コンソールに出力
            parameters=[  # パラメータ設定
            ]
        )
    ])
