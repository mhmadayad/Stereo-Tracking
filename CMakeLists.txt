cmake_minimum_required(VERSION 2.8.11)

project(TrackingStereo CXX)
set (CMAKE_CXX_STANDARD 14)
set (CMAKE_BUILD_TYPE Debug)

set(CMAKE_INCLUDE_CURRENT_DIR ON)

find_package(OpenCV 3.3.0 REQUIRED)
include_directories($(OpenCV_INCLUDE_DIRS))
include_directories(include)

add_executable(calibrate src/calib_single.cpp include/popt_pp.h)
target_link_libraries(calibrate ${OpenCV_LIBS} "-lpopt")

add_executable(read src/read_images.cpp)
target_link_libraries(read ${OpenCV_LIBS} "-lpopt")

add_executable(calibrate_stereo src/calib_stereo.cpp include/Ball.h)
target_link_libraries(calibrate_stereo ${OpenCV_LIBS} "-lpopt")

add_executable(undistort_rectify src/undistort_rectify.cpp)
target_link_libraries(undistort_rectify ${OpenCV_LIBS} "-lpopt")

add_executable(detection src/circle_detection.cpp src/Ball.cpp include/Constants.h include/Ball.h)
target_link_libraries(detection ${OpenCV_LIBS})

add_executable(matching src/stereo_matching.cpp src/Ball.cpp include/Constants.h include/Ball.h)
target_link_libraries(matching ${OpenCV_LIBS} "-lpopt")
