import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import LoadComposableNodes, Node
from launch_ros.descriptions import ComposableNode


def generate_launch_description():

    def get_radar_detect_fly_node(package, plugin):
        return ComposableNode(
            package=package,
            plugin=plugin,
            name='radar_detect_fly_node',
            extra_arguments=[{'use_intra_process_comms': True}],
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
    radar_base_launch_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory('tdt_vision'),
                'launch',
                'radar_base.launch.py',
            )
        ]),
    )
    radar_detect_fly_node = get_radar_detect_fly_node(
        'tdt_vision', 'tdt_radar::DetectFly')
    load_radar_detect_fly_node = LoadComposableNodes(
        target_container='/camera_detector_container',
        composable_node_descriptions=[radar_detect_fly_node],
    )
    lock_node = get_lock_node('tdt_lock', 'lock_node')

    return LaunchDescription([
            radar_base_launch_cmd,
            load_radar_detect_fly_node,
            lock_node,
        ])
