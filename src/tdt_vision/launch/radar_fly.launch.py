from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    def get_radar_detect_fly_node(package, executable):
        return Node(
            package=package,
            executable=executable,
            name='radar_detect_fly_node',
            output='screen',
            emulate_tty=True,
        )

    def get_lock_node(package, executable):
        return Node(
            package=package,
            executable=executable,
            name='lock_node',
            output='screen',
            emulate_tty=True,
        )

    # 创建节点描述
    radar_detect_fly_node = get_radar_detect_fly_node('tdt_vision', 'radar_detect_fly_node')
    lock_node = get_lock_node('tdt_lock', 'lock_node')

    return LaunchDescription([
            radar_detect_fly_node,
            lock_node,
        ])
