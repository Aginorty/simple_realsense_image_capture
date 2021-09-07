cmake_minimum_required(VERSION 3.10)
project(SimpleCapture)
find_package(OpenCV REQUIRED)

add_executable(SimpleCapture realsense_simple_capture.cpp)
target_link_libraries(SimpleCapture PUBLIC realsense2 ${OpenCV_LIBS})