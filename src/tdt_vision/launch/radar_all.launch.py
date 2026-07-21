from launch import LaunchDescription
from launch.actions import Shutdown
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    common_extra_arguments = [{"use_intra_process_comms": True}]

    camera_node = ComposableNode(
        package="tdt_vision",
        plugin="tdt_vision::TDTCameraNode",
        name="camera_node",
        parameters=[{"config_path": "./config/config.json", "auto_start": True}],
        extra_arguments=common_extra_arguments,
    )

    foxglove_node = ComposableNode(
        package="foxglove_bridge",
        plugin="foxglove_bridge::FoxgloveBridge",
        name="foxglove_bridge_node",
        parameters=[{"send_buffer_limit": 1000000000}],
        extra_arguments=common_extra_arguments,
    )

    debug_node = ComposableNode(
        package="tdt_vision",
        plugin="tdt_vision::NodeDebug",
        name="debug_node",
        extra_arguments=common_extra_arguments,
    )

    radar_detect_node = ComposableNode(
        package="tdt_vision",
        plugin="tdt_radar::Detect",
        name="radar_detect_node",
        extra_arguments=common_extra_arguments,
    )

    radar_resolve_node = ComposableNode(
        package="tdt_vision",
        plugin="tdt_radar::Resolve",
        name="radar_resolve_node",
        extra_arguments=common_extra_arguments,
    )

    radar_detect_fly_node = ComposableNode(
        package="tdt_vision",
        plugin="tdt_radar::DetectFly",
        name="radar_detect_fly_node",
        extra_arguments=common_extra_arguments,
    )

    lock_node = ComposableNode(
        package="tdt_lock",
        plugin="tdt_lock::Lock",
        name="lock_node",
        extra_arguments=common_extra_arguments,
    )

    record_node = Node(
        package="databag_tool",
        executable="BagRecorderNode",
        name="record_node",
        output="both",
    )

    radar_container = ComposableNodeContainer(
        name="camera_detector_container",
        namespace="",
        package="rclcpp_components",
        executable="component_container_mt",
        parameters=[{"thread_num": 4}],
        composable_node_descriptions=[
            camera_node,
            foxglove_node,
            # debug_node,
            radar_detect_node,
            radar_resolve_node,
            radar_detect_fly_node,
            lock_node,
        ],
        output="both",
        emulate_tty=True,
        on_exit=Shutdown(),
    )

    return LaunchDescription([radar_container])
