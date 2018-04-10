cmake_minimum_required(VERSION 2.8.11)

project(TrackingStereo CXX)
set (CMAKE_CXX_STANDARD 14)
set (CMAKE_BUILD_TYPE Debug)

set(CMAKE_INCLUDE_CURRENT_DIR ON)

find_package(PCL 1.2 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

find_package(OpenCV 3.3.0 REQUIRED)
include_directories($(OpenCV_INCLUDE_DIRS))
include_directories(include)

find_package(OpenGL REQUIRED)

link_directories(${OpenGL_LIBRARY_DIRS})

find_package(PythonLibs 2.7)

add_executable(calibrate src/calib_single.cpp include/popt_pp.h)
target_link_libraries(calibrate ${OpenCV_LIBS} "-lpopt")

add_executable(read src/read_images.cpp)
target_link_libraries(read ${OpenCV_LIBS} ${PCL_LIBRARIES} ${GTK3_LIBRARIES} "-lpopt")

add_executable(calibrate_stereo src/calib_stereo.cpp include/Ball.h )
target_link_libraries(calibrate_stereo ${OpenCV_LIBS} ${GTK3_LIBRARIES} "-lpopt")

add_executable(undistort_rectify src/undistort_rectify.cpp src/Ball.cpp include/Ball.h)
target_link_libraries(undistort_rectify ${OpenCV_LIBS}  "-lpopt")

add_executable(main src/main.cpp src/LED.cpp include/LED.h include/Constants.h src/StereoMatching.cpp include/StereoMatching.h)
target_link_libraries(main ${PCL_LIBRARIES} ${OpenCV_LIBS})

add_executable(matching src/stereo_matching.cpp src/Ball.cpp src/StereoMatching.cpp include/Constants.h include/Ball.h include/StereoMatching.h)
target_link_libraries(matching ${OpenCV_LIBS} ${GTK3_LIBRARIES} "-lpopt")

add_executable(detection src/LEDDetection.cpp src/Ball.cpp src/StereoMatching.cpp include/Constants.h include/Ball.h include/StereoMatching.h)
target_link_libraries(detection ${OpenCV_LIBS} ${OPENGL_LIBRARIES} ${PCL_LIBRARIES} "-lpopt" "-lGL" "-lGLU" "-lglut")


