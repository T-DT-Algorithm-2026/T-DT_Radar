cmake_minimum_required(VERSION 3.8)
project(tdt_lock)

# 1. 显式查找 OpenCV，确保包含 calib3d 模块
find_package(OpenCV REQUIRED COMPONENTS core calib3d imgproc)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

include_directories(include ${OpenCV_INCLUDE_DIRS})

set(LOCK "lock")
file(GLOB_RECURSE LOCK_SRCS "src/*.cpp")

# 2. 添加库
ament_auto_add_library(${LOCK} SHARED
    ${LOCK_SRCS}
)

# 3. 显式链接 OpenCV 库到你的目标库中
target_link_libraries(${LOCK} 
    ${OpenCV_LIBRARIES}
    ${PCL_LIBRARIES}
)

# 4. 注册组件
rclcpp_components_register_node(${LOCK}
    PLUGIN "tdt_lock::Lock"
    EXECUTABLE ${LOCK}_node
)

ament_auto_package()