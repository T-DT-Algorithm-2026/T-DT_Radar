from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import LoadComposableNodes
from launch_ros.descriptions import ComposableNode


def generate_launch_description():

    def get_radar_detect_fly_node(package, plugin):
        return ComposableNode(
            package=package,
            plugin=plugin,
            name='radar_detect_fly_node',
            extra_arguments=[{'use_intra_process_comms': True}]
        )

    def get_lock_node(package, plugin):
        return ComposableNode(
            package=package,
            plugin=plugin,
            name='lock_node',
            extra_arguments=[{'use_intra_process_comms': True}]
        )

    def get_radar_loader(radar_detect_fly_node, lock_node):
        return TimerAction(
            period=1.0,
            actions=[
                LoadComposableNodes(
                    target_container='camera_detector_container',
                    composable_node_descriptions=[
                        #变向设置启动顺序
                        radar_detect_fly_node,
                        lock_node,
                    ],
                )
            ],
        )

    # 创建节点描述
    radar_detect_fly_node = get_radar_detect_fly_node('tdt_vision', 'tdt_radar::DetectFly')
    lock_node = get_lock_node('tdt_lock', 'tdt_lock::Lock')

    # 加载到已有的相机检测容器中
    radar_loader = get_radar_loader(radar_detect_fly_node, lock_node)
    return LaunchDescription([
            radar_loader,
        ])
