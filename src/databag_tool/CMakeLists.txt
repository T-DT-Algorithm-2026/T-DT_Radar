cmake_minimum_required(VERSION 3.8)
project(databag_tool)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 设置 C++ 标准为 C++14，并要求项目必须使用该标准
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON) # 要求项目必须使用设置的 C++ 标准，如果编译器不支持该标准，CMake 会报错

# 禁用编译器特定的 C++ 语言扩展
set(CMAKE_CXX_EXTENSIONS OFF)

# 为编译器添加编译选项，-pthread（启用多线程支持），-fexceptions（启用异常支持），-O3（启用最高级别的编译器优化）
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -pthread -fexceptions")

# 添加预处理器宏定义 ROOT_DIR
add_definitions(-DROOT_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}/\")

#为 C 编译器添加编译选项 -fexceptions，表示启用异常支持
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fexceptions" ) 

find_package (yaml-cpp REQUIRED)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()
find_package(rosidl_typesupport_cpp REQUIRED)

# find_path(MATPLOTLIB_CPP_INCLUDE_DIRS "matplotlibcpp.h")


include_directories(
  ${PYTHON_INCLUDE_DIRS}
  include)
  
ament_auto_add_executable(record
                          src/record.cpp
                         )
target_link_libraries(record
    ${PYTHON_LIBRARIES}
)
target_include_directories(record PRIVATE ${PYTHON_INCLUDE_DIRS})


ament_auto_add_executable(merge_bags
                          src/merge_bags.cpp
                         )
target_link_libraries(merge_bags
    ${PYTHON_LIBRARIES}
)
target_include_directories(merge_bags PRIVATE ${PYTHON_INCLUDE_DIRS})


ament_auto_add_library(BagRecorderNodeLibrary SHARED
  src/BagRecorderNodeWork.cpp

)

target_include_directories(BagRecorderNodeLibrary PUBLIC 
                           ${PYTHON_INCLUDE_DIRS} 
                           ${YAML_CPP_INCLUDE_DIRS} 
                           
                          )
target_link_libraries(BagRecorderNodeLibrary 
   ${PYTHON_LIBRARIES} ${YAML_CPP_LIBRARIES}   
)
rclcpp_components_register_node(BagRecorderNodeLibrary
  PLUGIN "BagRecorderNode"
  EXECUTABLE BagRecorderNode
)


install(TARGETS merge_bags
  DESTINATION lib/${PROJECT_NAME})
install(TARGETS record
  DESTINATION lib/${PROJECT_NAME})


ament_auto_package()
