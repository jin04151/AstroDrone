cmake_minimum_required(VERSION 3.10)
project(drone_vision_tensorrt LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(OpenCV REQUIRED)
find_package(CUDA REQUIRED)
find_package(Threads REQUIRED)

find_path(TENSORRT_INCLUDE NvInfer.h HINTS /usr/include/aarch64-linux-gnu /usr/include)
find_path(ONNXPARSER_INCLUDE NvOnnxParser.h HINTS /usr/include/aarch64-linux-gnu /usr/include)
find_library(NVINFER_LIB nvinfer HINTS /usr/lib/aarch64-linux-gnu)
find_library(ONNXPARSER_LIB nvonnxparser HINTS /usr/lib/aarch64-linux-gnu)
find_library(NVINFER_PLUGIN_LIB nvinfer_plugin HINTS /usr/lib/aarch64-linux-gnu)
if(NOT TENSORRT_INCLUDE OR NOT ONNXPARSER_INCLUDE OR NOT NVINFER_LIB OR NOT ONNXPARSER_LIB OR NOT NVINFER_PLUGIN_LIB)
    message(FATAL_ERROR "Jetson TensorRT headers/libraries not found; build on a JetPack host")
endif()

add_executable(jetson_yolo_trt
    main.cpp
    frame_source.cpp
    tensorrt_engine.cpp
)
target_include_directories(jetson_yolo_trt PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${OpenCV_INCLUDE_DIRS}
    ${CUDA_INCLUDE_DIRS}
    ${TENSORRT_INCLUDE}
    ${ONNXPARSER_INCLUDE}
)
target_link_libraries(jetson_yolo_trt PRIVATE
    ${OpenCV_LIBS}
    ${CUDA_LIBRARIES}
    ${NVINFER_LIB}
    ${ONNXPARSER_LIB}
    ${NVINFER_PLUGIN_LIB}
    Threads::Threads
)
target_compile_options(jetson_yolo_trt PRIVATE -Wall -Wextra)
