cmake_minimum_required(VERSION 3.10)
cmake_policy(SET CMP0079 NEW)

project(infer)
add_definitions(-std=c++17)
add_definitions(-DAPI_EXPORTS)
option(CUDA_USE_STATIC_CUDA_RUNTIME ON)
set(CMAKE_CXX_STANDARD 17)
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-deprecated-declarations")
# TODO(Call for PR): make cmake compatible with Windows
include(../cmake/modules/libnvinferConfig.cmake)

include_directories(${nvinfer_INCLUDE_DIRS})
include_directories(${PROJECT_SOURCE_DIR}/include)

find_package(OpenCV REQUIRED)

file(GLOB_RECURSE SRCS ${PROJECT_SOURCE_DIR}/src/*.cpp ${PROJECT_SOURCE_DIR}/src/*.cu)
add_library(infer  ${SRCS})
target_link_libraries(infer ${nvinfer_LIBRARY} ${nvonnxparser_LIBRARY} CUDA::cudart ${OpenCV_LIBS})
