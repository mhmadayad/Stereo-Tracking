# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.5

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:


#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:


# Remove some rules from gmake that .SUFFIXES does not remove.
SUFFIXES =

.SUFFIXES: .hpux_make_needs_suffix_list


# Suppress display of executed commands.
$(VERBOSE).SILENT:


# A target that is always out of date.
cmake_force:

.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /usr/bin/cmake

# The command to remove a file.
RM = /usr/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/moh/Desktop/Thesis/Task1/Stereo-Tracking

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/moh/Desktop/Thesis/Task1/Stereo-Tracking

# Include any dependencies generated for this target.
include CMakeFiles/calibrate.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/calibrate.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/calibrate.dir/flags.make

CMakeFiles/calibrate.dir/src/calib_single.cpp.o: CMakeFiles/calibrate.dir/flags.make
CMakeFiles/calibrate.dir/src/calib_single.cpp.o: src/calib_single.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/moh/Desktop/Thesis/Task1/Stereo-Tracking/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/calibrate.dir/src/calib_single.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/calibrate.dir/src/calib_single.cpp.o -c /home/moh/Desktop/Thesis/Task1/Stereo-Tracking/src/calib_single.cpp

CMakeFiles/calibrate.dir/src/calib_single.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/calibrate.dir/src/calib_single.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/moh/Desktop/Thesis/Task1/Stereo-Tracking/src/calib_single.cpp > CMakeFiles/calibrate.dir/src/calib_single.cpp.i

CMakeFiles/calibrate.dir/src/calib_single.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/calibrate.dir/src/calib_single.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/moh/Desktop/Thesis/Task1/Stereo-Tracking/src/calib_single.cpp -o CMakeFiles/calibrate.dir/src/calib_single.cpp.s

CMakeFiles/calibrate.dir/src/calib_single.cpp.o.requires:

.PHONY : CMakeFiles/calibrate.dir/src/calib_single.cpp.o.requires

CMakeFiles/calibrate.dir/src/calib_single.cpp.o.provides: CMakeFiles/calibrate.dir/src/calib_single.cpp.o.requires
	$(MAKE) -f CMakeFiles/calibrate.dir/build.make CMakeFiles/calibrate.dir/src/calib_single.cpp.o.provides.build
.PHONY : CMakeFiles/calibrate.dir/src/calib_single.cpp.o.provides

CMakeFiles/calibrate.dir/src/calib_single.cpp.o.provides.build: CMakeFiles/calibrate.dir/src/calib_single.cpp.o


# Object files for target calibrate
calibrate_OBJECTS = \
"CMakeFiles/calibrate.dir/src/calib_single.cpp.o"

# External object files for target calibrate
calibrate_EXTERNAL_OBJECTS =

calibrate: CMakeFiles/calibrate.dir/src/calib_single.cpp.o
calibrate: CMakeFiles/calibrate.dir/build.make
calibrate: /usr/local/lib/libopencv_stitching.so.3.3.0
calibrate: /usr/local/lib/libopencv_superres.so.3.3.0
calibrate: /usr/local/lib/libopencv_videostab.so.3.3.0
calibrate: /usr/local/lib/libopencv_aruco.so.3.3.0
calibrate: /usr/local/lib/libopencv_bgsegm.so.3.3.0
calibrate: /usr/local/lib/libopencv_bioinspired.so.3.3.0
calibrate: /usr/local/lib/libopencv_ccalib.so.3.3.0
calibrate: /usr/local/lib/libopencv_dpm.so.3.3.0
calibrate: /usr/local/lib/libopencv_face.so.3.3.0
calibrate: /usr/local/lib/libopencv_freetype.so.3.3.0
calibrate: /usr/local/lib/libopencv_fuzzy.so.3.3.0
calibrate: /usr/local/lib/libopencv_hdf.so.3.3.0
calibrate: /usr/local/lib/libopencv_img_hash.so.3.3.0
calibrate: /usr/local/lib/libopencv_line_descriptor.so.3.3.0
calibrate: /usr/local/lib/libopencv_optflow.so.3.3.0
calibrate: /usr/local/lib/libopencv_reg.so.3.3.0
calibrate: /usr/local/lib/libopencv_rgbd.so.3.3.0
calibrate: /usr/local/lib/libopencv_saliency.so.3.3.0
calibrate: /usr/local/lib/libopencv_stereo.so.3.3.0
calibrate: /usr/local/lib/libopencv_structured_light.so.3.3.0
calibrate: /usr/local/lib/libopencv_surface_matching.so.3.3.0
calibrate: /usr/local/lib/libopencv_tracking.so.3.3.0
calibrate: /usr/local/lib/libopencv_xfeatures2d.so.3.3.0
calibrate: /usr/local/lib/libopencv_ximgproc.so.3.3.0
calibrate: /usr/local/lib/libopencv_xobjdetect.so.3.3.0
calibrate: /usr/local/lib/libopencv_xphoto.so.3.3.0
calibrate: /usr/local/lib/libopencv_shape.so.3.3.0
calibrate: /usr/local/lib/libopencv_photo.so.3.3.0
calibrate: /usr/local/lib/libopencv_calib3d.so.3.3.0
calibrate: /usr/local/lib/libopencv_viz.so.3.3.0
calibrate: /usr/local/lib/libopencv_phase_unwrapping.so.3.3.0
calibrate: /usr/local/lib/libopencv_dnn.so.3.3.0
calibrate: /usr/local/lib/libopencv_video.so.3.3.0
calibrate: /usr/local/lib/libopencv_datasets.so.3.3.0
calibrate: /usr/local/lib/libopencv_plot.so.3.3.0
calibrate: /usr/local/lib/libopencv_text.so.3.3.0
calibrate: /usr/local/lib/libopencv_features2d.so.3.3.0
calibrate: /usr/local/lib/libopencv_flann.so.3.3.0
calibrate: /usr/local/lib/libopencv_highgui.so.3.3.0
calibrate: /usr/local/lib/libopencv_ml.so.3.3.0
calibrate: /usr/local/lib/libopencv_videoio.so.3.3.0
calibrate: /usr/local/lib/libopencv_imgcodecs.so.3.3.0
calibrate: /usr/local/lib/libopencv_objdetect.so.3.3.0
calibrate: /usr/local/lib/libopencv_imgproc.so.3.3.0
calibrate: /usr/local/lib/libopencv_core.so.3.3.0
calibrate: CMakeFiles/calibrate.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/moh/Desktop/Thesis/Task1/Stereo-Tracking/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable calibrate"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/calibrate.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/calibrate.dir/build: calibrate

.PHONY : CMakeFiles/calibrate.dir/build

CMakeFiles/calibrate.dir/requires: CMakeFiles/calibrate.dir/src/calib_single.cpp.o.requires

.PHONY : CMakeFiles/calibrate.dir/requires

CMakeFiles/calibrate.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/calibrate.dir/cmake_clean.cmake
.PHONY : CMakeFiles/calibrate.dir/clean

CMakeFiles/calibrate.dir/depend:
	cd /home/moh/Desktop/Thesis/Task1/Stereo-Tracking && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/moh/Desktop/Thesis/Task1/Stereo-Tracking /home/moh/Desktop/Thesis/Task1/Stereo-Tracking /home/moh/Desktop/Thesis/Task1/Stereo-Tracking /home/moh/Desktop/Thesis/Task1/Stereo-Tracking /home/moh/Desktop/Thesis/Task1/Stereo-Tracking/CMakeFiles/calibrate.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/calibrate.dir/depend

