# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.5

# Default target executed when no arguments are given to make.
default_target: all

.PHONY : default_target

# Allow only one "make -f Makefile2" at a time, but pass parallelism.
.NOTPARALLEL:


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

#=============================================================================
# Targets provided globally by CMake.

# Special rule for the target edit_cache
edit_cache:
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --cyan "Running CMake cache editor..."
	/usr/bin/cmake-gui -H$(CMAKE_SOURCE_DIR) -B$(CMAKE_BINARY_DIR)
.PHONY : edit_cache

# Special rule for the target edit_cache
edit_cache/fast: edit_cache

.PHONY : edit_cache/fast

# Special rule for the target rebuild_cache
rebuild_cache:
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --cyan "Running CMake to regenerate build system..."
	/usr/bin/cmake -H$(CMAKE_SOURCE_DIR) -B$(CMAKE_BINARY_DIR)
.PHONY : rebuild_cache

# Special rule for the target rebuild_cache
rebuild_cache/fast: rebuild_cache

.PHONY : rebuild_cache/fast

# The main all target
all: cmake_check_build_system
	$(CMAKE_COMMAND) -E cmake_progress_start /home/moh/Desktop/Thesis/Task1/Stereo-Tracking/CMakeFiles /home/moh/Desktop/Thesis/Task1/Stereo-Tracking/CMakeFiles/progress.marks
	$(MAKE) -f CMakeFiles/Makefile2 all
	$(CMAKE_COMMAND) -E cmake_progress_start /home/moh/Desktop/Thesis/Task1/Stereo-Tracking/CMakeFiles 0
.PHONY : all

# The main clean target
clean:
	$(MAKE) -f CMakeFiles/Makefile2 clean
.PHONY : clean

# The main clean target
clean/fast: clean

.PHONY : clean/fast

# Prepare targets for installation.
preinstall: all
	$(MAKE) -f CMakeFiles/Makefile2 preinstall
.PHONY : preinstall

# Prepare targets for installation.
preinstall/fast:
	$(MAKE) -f CMakeFiles/Makefile2 preinstall
.PHONY : preinstall/fast

# clear depends
depend:
	$(CMAKE_COMMAND) -H$(CMAKE_SOURCE_DIR) -B$(CMAKE_BINARY_DIR) --check-build-system CMakeFiles/Makefile.cmake 1
.PHONY : depend

#=============================================================================
# Target rules for targets named read

# Build rule for target.
read: cmake_check_build_system
	$(MAKE) -f CMakeFiles/Makefile2 read
.PHONY : read

# fast build rule for target.
read/fast:
	$(MAKE) -f CMakeFiles/read.dir/build.make CMakeFiles/read.dir/build
.PHONY : read/fast

#=============================================================================
# Target rules for targets named undistort_rectify

# Build rule for target.
undistort_rectify: cmake_check_build_system
	$(MAKE) -f CMakeFiles/Makefile2 undistort_rectify
.PHONY : undistort_rectify

# fast build rule for target.
undistort_rectify/fast:
	$(MAKE) -f CMakeFiles/undistort_rectify.dir/build.make CMakeFiles/undistort_rectify.dir/build
.PHONY : undistort_rectify/fast

#=============================================================================
# Target rules for targets named calibrate_stereo

# Build rule for target.
calibrate_stereo: cmake_check_build_system
	$(MAKE) -f CMakeFiles/Makefile2 calibrate_stereo
.PHONY : calibrate_stereo

# fast build rule for target.
calibrate_stereo/fast:
	$(MAKE) -f CMakeFiles/calibrate_stereo.dir/build.make CMakeFiles/calibrate_stereo.dir/build
.PHONY : calibrate_stereo/fast

#=============================================================================
# Target rules for targets named calibrate

# Build rule for target.
calibrate: cmake_check_build_system
	$(MAKE) -f CMakeFiles/Makefile2 calibrate
.PHONY : calibrate

# fast build rule for target.
calibrate/fast:
	$(MAKE) -f CMakeFiles/calibrate.dir/build.make CMakeFiles/calibrate.dir/build
.PHONY : calibrate/fast

#=============================================================================
# Target rules for targets named detection

# Build rule for target.
detection: cmake_check_build_system
	$(MAKE) -f CMakeFiles/Makefile2 detection
.PHONY : detection

# fast build rule for target.
detection/fast:
	$(MAKE) -f CMakeFiles/detection.dir/build.make CMakeFiles/detection.dir/build
.PHONY : detection/fast

src/calib_single.o: src/calib_single.cpp.o

.PHONY : src/calib_single.o

# target to build an object file
src/calib_single.cpp.o:
	$(MAKE) -f CMakeFiles/calibrate.dir/build.make CMakeFiles/calibrate.dir/src/calib_single.cpp.o
.PHONY : src/calib_single.cpp.o

src/calib_single.i: src/calib_single.cpp.i

.PHONY : src/calib_single.i

# target to preprocess a source file
src/calib_single.cpp.i:
	$(MAKE) -f CMakeFiles/calibrate.dir/build.make CMakeFiles/calibrate.dir/src/calib_single.cpp.i
.PHONY : src/calib_single.cpp.i

src/calib_single.s: src/calib_single.cpp.s

.PHONY : src/calib_single.s

# target to generate assembly for a file
src/calib_single.cpp.s:
	$(MAKE) -f CMakeFiles/calibrate.dir/build.make CMakeFiles/calibrate.dir/src/calib_single.cpp.s
.PHONY : src/calib_single.cpp.s

src/calib_stereo.o: src/calib_stereo.cpp.o

.PHONY : src/calib_stereo.o

# target to build an object file
src/calib_stereo.cpp.o:
	$(MAKE) -f CMakeFiles/calibrate_stereo.dir/build.make CMakeFiles/calibrate_stereo.dir/src/calib_stereo.cpp.o
.PHONY : src/calib_stereo.cpp.o

src/calib_stereo.i: src/calib_stereo.cpp.i

.PHONY : src/calib_stereo.i

# target to preprocess a source file
src/calib_stereo.cpp.i:
	$(MAKE) -f CMakeFiles/calibrate_stereo.dir/build.make CMakeFiles/calibrate_stereo.dir/src/calib_stereo.cpp.i
.PHONY : src/calib_stereo.cpp.i

src/calib_stereo.s: src/calib_stereo.cpp.s

.PHONY : src/calib_stereo.s

# target to generate assembly for a file
src/calib_stereo.cpp.s:
	$(MAKE) -f CMakeFiles/calibrate_stereo.dir/build.make CMakeFiles/calibrate_stereo.dir/src/calib_stereo.cpp.s
.PHONY : src/calib_stereo.cpp.s

src/circle_detection.o: src/circle_detection.cpp.o

.PHONY : src/circle_detection.o

# target to build an object file
src/circle_detection.cpp.o:
	$(MAKE) -f CMakeFiles/detection.dir/build.make CMakeFiles/detection.dir/src/circle_detection.cpp.o
.PHONY : src/circle_detection.cpp.o

src/circle_detection.i: src/circle_detection.cpp.i

.PHONY : src/circle_detection.i

# target to preprocess a source file
src/circle_detection.cpp.i:
	$(MAKE) -f CMakeFiles/detection.dir/build.make CMakeFiles/detection.dir/src/circle_detection.cpp.i
.PHONY : src/circle_detection.cpp.i

src/circle_detection.s: src/circle_detection.cpp.s

.PHONY : src/circle_detection.s

# target to generate assembly for a file
src/circle_detection.cpp.s:
	$(MAKE) -f CMakeFiles/detection.dir/build.make CMakeFiles/detection.dir/src/circle_detection.cpp.s
.PHONY : src/circle_detection.cpp.s

src/read_images.o: src/read_images.cpp.o

.PHONY : src/read_images.o

# target to build an object file
src/read_images.cpp.o:
	$(MAKE) -f CMakeFiles/read.dir/build.make CMakeFiles/read.dir/src/read_images.cpp.o
.PHONY : src/read_images.cpp.o

src/read_images.i: src/read_images.cpp.i

.PHONY : src/read_images.i

# target to preprocess a source file
src/read_images.cpp.i:
	$(MAKE) -f CMakeFiles/read.dir/build.make CMakeFiles/read.dir/src/read_images.cpp.i
.PHONY : src/read_images.cpp.i

src/read_images.s: src/read_images.cpp.s

.PHONY : src/read_images.s

# target to generate assembly for a file
src/read_images.cpp.s:
	$(MAKE) -f CMakeFiles/read.dir/build.make CMakeFiles/read.dir/src/read_images.cpp.s
.PHONY : src/read_images.cpp.s

src/undistort_rectify.o: src/undistort_rectify.cpp.o

.PHONY : src/undistort_rectify.o

# target to build an object file
src/undistort_rectify.cpp.o:
	$(MAKE) -f CMakeFiles/undistort_rectify.dir/build.make CMakeFiles/undistort_rectify.dir/src/undistort_rectify.cpp.o
.PHONY : src/undistort_rectify.cpp.o

src/undistort_rectify.i: src/undistort_rectify.cpp.i

.PHONY : src/undistort_rectify.i

# target to preprocess a source file
src/undistort_rectify.cpp.i:
	$(MAKE) -f CMakeFiles/undistort_rectify.dir/build.make CMakeFiles/undistort_rectify.dir/src/undistort_rectify.cpp.i
.PHONY : src/undistort_rectify.cpp.i

src/undistort_rectify.s: src/undistort_rectify.cpp.s

.PHONY : src/undistort_rectify.s

# target to generate assembly for a file
src/undistort_rectify.cpp.s:
	$(MAKE) -f CMakeFiles/undistort_rectify.dir/build.make CMakeFiles/undistort_rectify.dir/src/undistort_rectify.cpp.s
.PHONY : src/undistort_rectify.cpp.s

# Help Target
help:
	@echo "The following are some of the valid targets for this Makefile:"
	@echo "... all (the default if no target is provided)"
	@echo "... clean"
	@echo "... depend"
	@echo "... edit_cache"
	@echo "... read"
	@echo "... undistort_rectify"
	@echo "... calibrate_stereo"
	@echo "... rebuild_cache"
	@echo "... calibrate"
	@echo "... detection"
	@echo "... src/calib_single.o"
	@echo "... src/calib_single.i"
	@echo "... src/calib_single.s"
	@echo "... src/calib_stereo.o"
	@echo "... src/calib_stereo.i"
	@echo "... src/calib_stereo.s"
	@echo "... src/circle_detection.o"
	@echo "... src/circle_detection.i"
	@echo "... src/circle_detection.s"
	@echo "... src/read_images.o"
	@echo "... src/read_images.i"
	@echo "... src/read_images.s"
	@echo "... src/undistort_rectify.o"
	@echo "... src/undistort_rectify.i"
	@echo "... src/undistort_rectify.s"
.PHONY : help



#=============================================================================
# Special targets to cleanup operation of make.

# Special rule to run CMake to check the build system integrity.
# No rule that depends on this can have commands that come from listfiles
# because they might be regenerated.
cmake_check_build_system:
	$(CMAKE_COMMAND) -H$(CMAKE_SOURCE_DIR) -B$(CMAKE_BINARY_DIR) --check-build-system CMakeFiles/Makefile.cmake 0
.PHONY : cmake_check_build_system

