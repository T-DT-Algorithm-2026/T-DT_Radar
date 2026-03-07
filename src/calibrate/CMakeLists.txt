cmake_minimum_required(VERSION 3.8)
project(calibrate)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Ceres REQUIRED)

# Include directories
include_directories(
  calibrate_fly/include
  calibrate_fly_opencv/include
  ${OpenCV_INCLUDE_DIRS}
  ${CERES_INCLUDE_DIRS}
)

# calibrate_fly_opencv executable (OpenCV only)
add_executable(calibrate_fly_opencv_node
  calibrate_fly_opencv/src/calibrate_fly_opencv.cpp
)

target_link_libraries(calibrate_fly_opencv_node
  ${OpenCV_LIBS}
)

# calibrate_fly executable (OpenCV + Ceres)
add_executable(calibrate_fly_node
  calibrate_fly/src/calibrate_fly.cpp
)

target_link_libraries(calibrate_fly_node
  ${OpenCV_LIBS}
  ${CERES_LIBRARIES}
)

# Install executables
install(TARGETS 
  calibrate_fly_opencv_node
  calibrate_fly_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install include directories
install(DIRECTORY calibrate_fly/include/
  DESTINATION include
)

install(DIRECTORY calibrate_fly_opencv/include/
  DESTINATION include
)

ament_package()
