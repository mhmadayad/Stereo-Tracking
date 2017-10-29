cmake_minimum_required(VERSION 2.8.11)

project(CALIB_CAM CXX)
set (CMAKE_CXX_STANDARD 14)
set (CMAKE_BUILD_TYPE Debug)

set(CMAKE_INCLUDE_CURRENT_DIR ON)

find_package(OpenCV REQUIRED)
include_directories($(OpenCV_INCLUDE_DIRS))
include_directories(include)

add_executable(calibrate src/calib_single.cpp include/popt_pp.h)
target_link_libraries(calibrate ${OpenCV_LIBS} "-lpopt")


