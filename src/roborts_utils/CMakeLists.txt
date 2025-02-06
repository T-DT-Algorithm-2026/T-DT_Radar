cmake_minimum_required(VERSION 3.8)
project(roborts_utils)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

add_definitions(-w)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(base_interface REQUIRED)
find_package(vision_interface REQUIRED)

find_package(OpenCV 4 REQUIRED)

find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)

find_package(OpenMP REQUIRED)
find_package(jsoncpp REQUIRED)

find_package(yaml-cpp REQUIRED)
include_directories(${YAML_CPP_INCLUDE_DIRS})

find_package(Boost REQUIRED)
include_directories(${Boost_INCLUDE_DIRS})
link_directories(${Boost_LIBRARY_DIRS})
if(CMAKE_HOST_SYSTEM_NAME MATCHES "Windows")
  link_libraries(ws2_32)
endif()

include_directories(
  include/${PROJECT_NAME}
)

file(GLOB SRC_FILES "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp")
# aux_source_directory(src  DIR_SRCS)

# message(${DIR_SRCS}
# )

add_library(${PROJECT_NAME} SHARED 
# ${DIR_SRCS}
${SRC_FILES}
)

target_link_libraries(${PROJECT_NAME}
  -lpthread
  jsoncpp_lib
  ${YAML_CPP_LIBRARIES}
  ${Boost_LIBRARIES}
)

ament_target_dependencies(
  ${PROJECT_NAME}
  rclcpp
  std_msgs
  OpenCV
  Eigen3
  OpenMP
  jsoncpp
  Boost
  base_interface
  vision_interface
)

ament_export_targets(${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
        rclcpp
        std_msgs
        OpenCV
        Eigen3
        OpenMP
        jsoncpp
        Boost
        base_interface
        vision_interface
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

ament_package()
