# - Try to find nvinfer
#
# The following variables are optionally searched for defaults
#  nvinfer_ROOT_DIR:            Base directory where all nvinfer components are found
#
# The following are set after configuration is done:
#  nvinfer_FOUND
#  nvinfer_INCLUDE_DIRS
#  nvinfer_LIBRARIES

include(FindPackageHandleStandardArgs)

set(nvinfer_ROOT_DIR "" CACHE PATH "Folder contains nvinfer")

find_path(nvinfer_INCLUDE_DIR NvInfer.h
    HINTS ${nvinfer_ROOT_DIR} ENV TensorRT_ROOT
    PATH_SUFFIXES include)

find_library(nvinfer_LIBRARY nvinfer
    HINTS ${nvinfer_ROOT_DIR} ENV TensorRT_ROOT
    PATH_SUFFIXES lib lib64)

find_library(nvonnxparser_LIBRARY nvonnxparser
    HINTS ${nvinfer_ROOT_DIR} ENV TensorRT_ROOT
    PATH_SUFFIXES lib lib64)

find_package_handle_standard_args(nvinfer DEFAULT_MSG
    nvinfer_INCLUDE_DIR
    nvinfer_LIBRARY
    nvonnxparser_LIBRARY)

if(nvinfer_FOUND)
  set(nvinfer_INCLUDE_DIRS ${nvinfer_INCLUDE_DIR})
  set(nvinfer_LIBRARIES ${nvinfer_LIBRARY} ${nvonnxparser_LIBRARY})
  message(STATUS "Found nvinfer (include: ${nvinfer_INCLUDE_DIR}, libraries: ${nvinfer_LIBRARIES})")
  mark_as_advanced(nvinfer_ROOT_DIR nvinfer_LIBRARY nvonnxparser_LIBRARY nvinfer_INCLUDE_DIR)
endif()
