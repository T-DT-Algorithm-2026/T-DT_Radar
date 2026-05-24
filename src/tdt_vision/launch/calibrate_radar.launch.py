import os
import sys
import yaml
from ament_index_python.packages import get_package_share_directory

sys.path.append(os.path.join(get_package_share_directory('tdt_vision'), 'launch'))

from launch_ros.descriptions import ComposableNode
from launch_ros.actions import ComposableNodeContainer, Node
from launch.actions import TimerAction, Shutdown
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    def get_camera_node(package, plugin):
        return ComposableNode(
            package=package,
            plugin=plugin,
            name='camera_node',
            parameters=[{'config_path': './config/config.json', 'auto_start': True}],
            extra_arguments=[{'use_intra_process_comms': True}]
        )
        
    def get_radar_calib_node(package, plugin):
        return ComposableNode(
            package=package,
            plugin=plugin,
            name='radar_calib_node',
            extra_arguments=[{'use_intra_process_comms': True}]
        )        

    def get_camera_detector_container(camera_node,calib_node):
        return ComposableNodeContainer(
            name='camera_detector_container',
            namespace='',
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                #变向设置启动顺序
                camera_node,
                calib_node
            ],
            output='both',
            emulate_tty=True,
            on_exit=Shutdown(),
        )


    # 创建节点描述
    calib_node = get_radar_calib_node('tdt_vision', 'tdt_radar::Calibrate')

    camera_node = get_camera_node('tdt_vision', 'tdt_vision::TDTCameraNode')

    # 创建节点容器
    cam_detector = get_camera_detector_container(camera_node,calib_node)

    return LaunchDescription([
            cam_detector
        ])