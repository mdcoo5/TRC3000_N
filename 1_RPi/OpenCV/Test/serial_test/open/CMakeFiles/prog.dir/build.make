# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.0

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
CMAKE_SOURCE_DIR = /home/pi/TRC3000_N/1_RPi/OpenCV/Test/serial_test/open

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/pi/TRC3000_N/1_RPi/OpenCV/Test/serial_test/open

# Include any dependencies generated for this target.
include CMakeFiles/prog.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/prog.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/prog.dir/flags.make

CMakeFiles/prog.dir/RPI_TEST_OPENCV.cpp.o: CMakeFiles/prog.dir/flags.make
CMakeFiles/prog.dir/RPI_TEST_OPENCV.cpp.o: RPI_TEST_OPENCV.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/pi/TRC3000_N/1_RPi/OpenCV/Test/serial_test/open/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/prog.dir/RPI_TEST_OPENCV.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/prog.dir/RPI_TEST_OPENCV.cpp.o -c /home/pi/TRC3000_N/1_RPi/OpenCV/Test/serial_test/open/RPI_TEST_OPENCV.cpp

CMakeFiles/prog.dir/RPI_TEST_OPENCV.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/prog.dir/RPI_TEST_OPENCV.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/pi/TRC3000_N/1_RPi/OpenCV/Test/serial_test/open/RPI_TEST_OPENCV.cpp > CMakeFiles/prog.dir/RPI_TEST_OPENCV.cpp.i

CMakeFiles/prog.dir/RPI_TEST_OPENCV.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/prog.dir/RPI_TEST_OPENCV.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/pi/TRC3000_N/1_RPi/OpenCV/Test/serial_test/open/RPI_TEST_OPENCV.cpp -o CMakeFiles/prog.dir/RPI_TEST_OPENCV.cpp.s

CMakeFiles/prog.dir/RPI_TEST_OPENCV.cpp.o.requires:
.PHONY : CMakeFiles/prog.dir/RPI_TEST_OPENCV.cpp.o.requires

CMakeFiles/prog.dir/RPI_TEST_OPENCV.cpp.o.provides: CMakeFiles/prog.dir/RPI_TEST_OPENCV.cpp.o.requires
	$(MAKE) -f CMakeFiles/prog.dir/build.make CMakeFiles/prog.dir/RPI_TEST_OPENCV.cpp.o.provides.build
.PHONY : CMakeFiles/prog.dir/RPI_TEST_OPENCV.cpp.o.provides

CMakeFiles/prog.dir/RPI_TEST_OPENCV.cpp.o.provides.build: CMakeFiles/prog.dir/RPI_TEST_OPENCV.cpp.o

# Object files for target prog
prog_OBJECTS = \
"CMakeFiles/prog.dir/RPI_TEST_OPENCV.cpp.o"

# External object files for target prog
prog_EXTERNAL_OBJECTS =

prog: CMakeFiles/prog.dir/RPI_TEST_OPENCV.cpp.o
prog: CMakeFiles/prog.dir/build.make
prog: /usr/lib/arm-linux-gnueabihf/libopencv_videostab.so.2.4.9
prog: /usr/lib/arm-linux-gnueabihf/libopencv_video.so.2.4.9
prog: /usr/lib/arm-linux-gnueabihf/libopencv_ts.so.2.4.9
prog: /usr/lib/arm-linux-gnueabihf/libopencv_superres.so.2.4.9
prog: /usr/lib/arm-linux-gnueabihf/libopencv_stitching.so.2.4.9
prog: /usr/lib/arm-linux-gnueabihf/libopencv_photo.so.2.4.9
prog: /usr/lib/arm-linux-gnueabihf/libopencv_ocl.so.2.4.9
prog: /usr/lib/arm-linux-gnueabihf/libopencv_objdetect.so.2.4.9
prog: /usr/lib/arm-linux-gnueabihf/libopencv_ml.so.2.4.9
prog: /usr/lib/arm-linux-gnueabihf/libopencv_legacy.so.2.4.9
prog: /usr/lib/arm-linux-gnueabihf/libopencv_imgproc.so.2.4.9
prog: /usr/lib/arm-linux-gnueabihf/libopencv_highgui.so.2.4.9
prog: /usr/lib/arm-linux-gnueabihf/libopencv_gpu.so.2.4.9
prog: /usr/lib/arm-linux-gnueabihf/libopencv_flann.so.2.4.9
prog: /usr/lib/arm-linux-gnueabihf/libopencv_features2d.so.2.4.9
prog: /usr/lib/arm-linux-gnueabihf/libopencv_core.so.2.4.9
prog: /usr/lib/arm-linux-gnueabihf/libopencv_contrib.so.2.4.9
prog: /usr/lib/arm-linux-gnueabihf/libopencv_calib3d.so.2.4.9
prog: /usr/lib/arm-linux-gnueabihf/libopencv_photo.so.2.4.9
prog: /usr/lib/arm-linux-gnueabihf/libopencv_legacy.so.2.4.9
prog: /usr/lib/arm-linux-gnueabihf/libopencv_video.so.2.4.9
prog: /usr/lib/arm-linux-gnueabihf/libopencv_objdetect.so.2.4.9
prog: /usr/lib/arm-linux-gnueabihf/libopencv_ml.so.2.4.9
prog: /usr/lib/arm-linux-gnueabihf/libopencv_calib3d.so.2.4.9
prog: /usr/lib/arm-linux-gnueabihf/libopencv_features2d.so.2.4.9
prog: /usr/lib/arm-linux-gnueabihf/libopencv_highgui.so.2.4.9
prog: /usr/lib/arm-linux-gnueabihf/libopencv_imgproc.so.2.4.9
prog: /usr/lib/arm-linux-gnueabihf/libopencv_flann.so.2.4.9
prog: /usr/lib/arm-linux-gnueabihf/libopencv_core.so.2.4.9
prog: CMakeFiles/prog.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX executable prog"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/prog.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/prog.dir/build: prog
.PHONY : CMakeFiles/prog.dir/build

CMakeFiles/prog.dir/requires: CMakeFiles/prog.dir/RPI_TEST_OPENCV.cpp.o.requires
.PHONY : CMakeFiles/prog.dir/requires

CMakeFiles/prog.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/prog.dir/cmake_clean.cmake
.PHONY : CMakeFiles/prog.dir/clean

CMakeFiles/prog.dir/depend:
	cd /home/pi/TRC3000_N/1_RPi/OpenCV/Test/serial_test/open && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/pi/TRC3000_N/1_RPi/OpenCV/Test/serial_test/open /home/pi/TRC3000_N/1_RPi/OpenCV/Test/serial_test/open /home/pi/TRC3000_N/1_RPi/OpenCV/Test/serial_test/open /home/pi/TRC3000_N/1_RPi/OpenCV/Test/serial_test/open /home/pi/TRC3000_N/1_RPi/OpenCV/Test/serial_test/open/CMakeFiles/prog.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/prog.dir/depend

