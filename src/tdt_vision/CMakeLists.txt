cmake_minimum_required(VERSION 3.8)
project(tdt_vision)

set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_STANDARD 20)

set(CMAKE_POSITION_INDEPENDENT_CODE ON)

include_directories(  
    "detect/include"
    "detect_fly/include"
    "resolve/include"
    "calibrate/include"
    "calibrate_fly/include"
    "camera/include"
    "TDT_Camera/include"
    "record/include"
)

add_subdirectory(TDT_Camera)#tdt新相机
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

set(CAMERA_NODE "camera")   # 相机节点名称
file(GLOB_RECURSE CAMERA_SRCS "camera/src/tdt_camera_node.cpp")
ament_auto_add_library(${CAMERA_NODE} SHARED
    ${CAMERA_SRCS}
)
target_link_libraries(${CAMERA_NODE}
    TDT_Camera
)
rclcpp_components_register_node(${CAMERA_NODE}
    PLUGIN "tdt_vision::TDTCameraNode"
    EXECUTABLE ${CAMERA_NODE}_node
)

option(CUDA_USE_STATIC_CUDA_RUNTIME OFF)
set(CMAKE_BUILD_TYPE Debug)
set(CMAKE_CUDA_COMPILER /usr/local/cuda/bin/nvcc)
include_directories(/usr/local/cuda/include)
link_directories(/usr/local/cuda/lib64)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

add_subdirectory(utils)
include_directories(${PROJECT_SOURCE_DIR}/utils/include)

set(RADAR_DETECT "radar_detect")
file(GLOB_RECURSE RADAR_DETECT_SRCS "detect/src/*.cpp")
ament_auto_add_library(${RADAR_DETECT} SHARED
    ${RADAR_DETECT_SRCS}
)
find_package(CUDA)
include(${PROJECT_SOURCE_DIR}/cmake/modules/libnvinferConfig.cmake)
message("Using Cuda Version: ${CUDA_VERSION} and TensorRT Version: ${nvinfer_VERSION}")
add_definitions(-D use_tensorrt)
enable_language(CUDA)
include_directories(${CUDA_INCLUDE_DIRS})
include_directories(${nvinfer_INCLUDE_DIRS})

add_subdirectory(infer)
include_directories(${PROJECT_SOURCE_DIR}/infer/include)

target_link_libraries(${RADAR_DETECT}
    utils
    infer 
    ${nvinfer_LIBRARY} 
    cudart 
    nvonnxparser 
    fmt 
    ${PCL_LIBRARIES}
    )
rclcpp_components_register_node(${RADAR_DETECT}
    PLUGIN "tdt_radar::Detect"
    EXECUTABLE ${RADAR_DETECT}_node
)

set(RADAR_DETECT_FLY "radar_detect_fly")
file(GLOB_RECURSE RADAR_DETECT_FLY_SRCS "detect_fly/src/*.cpp")
ament_auto_add_library(${RADAR_DETECT_FLY} SHARED
    ${RADAR_DETECT_FLY_SRCS}
)
ament_target_dependencies(${RADAR_DETECT_FLY} 
    vision_interface 
)

target_link_libraries(${RADAR_DETECT_FLY}
    utils          
    infer          
    ${nvinfer_LIBRARY} 
    cudart         
    nvonnxparser   
    fmt            
)
rclcpp_components_register_node(${RADAR_DETECT_FLY}
    PLUGIN "tdt_radar::DetectFly"
    EXECUTABLE ${RADAR_DETECT_FLY}_node
)

set(RADAR_RESOLVE "radar_resolve")
file(GLOB_RECURSE RADAR_RESOLVE_SRCS "resolve/src/*.cpp")

ament_auto_add_library(${RADAR_RESOLVE} SHARED
    ${RADAR_RESOLVE_SRCS}
)
rclcpp_components_register_node(${RADAR_RESOLVE}
    PLUGIN "tdt_radar::Resolve"
    EXECUTABLE ${RADAR_RESOLVE}_node
)

set(RADAR_CALIBRATE "radar_calibrate")
file(GLOB_RECURSE RADAR_CALIBRATE_SRCS "calibrate/src/*.cpp")
ament_auto_add_library(${RADAR_CALIBRATE} SHARED
    ${RADAR_CALIBRATE_SRCS}
)
rclcpp_components_register_node(${RADAR_CALIBRATE}
    PLUGIN "tdt_radar::Calibrate"
    EXECUTABLE ${RADAR_CALIBRATE}_node
)

set(RADAR_CALIBRATE_FLY "radar_calibrate_fly")
file(GLOB_RECURSE RADAR_CALIBRATE_FLY_SRCS "calibrate_fly/src/*.cpp")
ament_auto_add_library(${RADAR_CALIBRATE_FLY} SHARED
    ${RADAR_CALIBRATE_FLY_SRCS}
)
rclcpp_components_register_node(${RADAR_CALIBRATE_FLY}
    PLUGIN "tdt_radar::CalibrateFly"
    EXECUTABLE ${RADAR_CALIBRATE_FLY}_node
)

set(RADAR_RECORD_NODE "radar_record") # radar_record节点名称
file(GLOB_RECURSE RADAR_RECORD_SRCS "record/src/*.cpp")
ament_auto_add_library(${RADAR_RECORD_NODE} SHARED
    ${RADAR_RECORD_SRCS}
)
rclcpp_components_register_node(${RADAR_RECORD_NODE}
    PLUGIN "tdt_radar::Record"
    EXECUTABLE ${RADAR_RECORD_NODE}_node
)


ament_auto_package(
    INSTALL_TO_SHARE
    launch
)
