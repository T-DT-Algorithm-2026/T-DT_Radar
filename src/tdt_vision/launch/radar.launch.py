import os
import sys
import yaml
from ament_index_python.packages import get_package_share_directory

sys.path.append(os.path.join(get_package_share_directory('tdt_vision'), 'launch'))

from launch_ros.descriptions import ComposableNode
from launch_ros.actions import LoadComposableNodes, Node
from launch.actions import TimerAction, Shutdown
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    def get_radar_detect_node(package, plugin):
        return ComposableNode(
            package=package,
            plugin=plugin,
            name='radar_detect_node',
            extra_arguments=[{'use_intra_process_comms': True}]
        )

    def get_radar_resolve_node(package, plugin):
        return ComposableNode(
            package=package,
            plugin=plugin,
            name='radar_resolve_node',
            extra_arguments=[{'use_intra_process_comms': True}]
        )

    def get_radar_loader(radar_detect_node, radar_resolve_node):
        return TimerAction(
            period=1.0,
            actions=[
                LoadComposableNodes(
                    target_container='camera_detector_container',
                    composable_node_descriptions=[
                        #变向设置启动顺序
                        radar_detect_node,
                        radar_resolve_node,
                    ],
                )
            ],
        )


    # 创建节点描述
    radar_detect_node = get_radar_detect_node('tdt_vision', 'tdt_radar::Detect')
    radar_resolve_node = get_radar_resolve_node('tdt_vision', 'tdt_radar::Resolve')


    # 创建节点容器
    radar_base_launch_cmd = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('tdt_vision'), 'launch', 'radar_base.launch.py')]),
             )
    radar_loader = get_radar_loader(radar_detect_node, radar_resolve_node)
    plugin_map_launch_cmd = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('tdt_vision'), 'launch', 'map_server_launch.py')]),
             )
    return LaunchDescription([
            radar_base_launch_cmd,
            radar_loader,
            plugin_map_launch_cmd,
        ])
