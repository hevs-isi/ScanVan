# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.7

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
CMAKE_SOURCE_DIR = /home/scanvandev/ScanVan/Calibration

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/scanvandev/ScanVan/Calibration

# Include any dependencies generated for this target.
include CMakeFiles/positionCalibration.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/positionCalibration.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/positionCalibration.dir/flags.make

CMakeFiles/positionCalibration.dir/src/positionCalibration.cpp.o: CMakeFiles/positionCalibration.dir/flags.make
CMakeFiles/positionCalibration.dir/src/positionCalibration.cpp.o: src/positionCalibration.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/scanvandev/ScanVan/Calibration/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/positionCalibration.dir/src/positionCalibration.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/positionCalibration.dir/src/positionCalibration.cpp.o -c /home/scanvandev/ScanVan/Calibration/src/positionCalibration.cpp

CMakeFiles/positionCalibration.dir/src/positionCalibration.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/positionCalibration.dir/src/positionCalibration.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/scanvandev/ScanVan/Calibration/src/positionCalibration.cpp > CMakeFiles/positionCalibration.dir/src/positionCalibration.cpp.i

CMakeFiles/positionCalibration.dir/src/positionCalibration.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/positionCalibration.dir/src/positionCalibration.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/scanvandev/ScanVan/Calibration/src/positionCalibration.cpp -o CMakeFiles/positionCalibration.dir/src/positionCalibration.cpp.s

CMakeFiles/positionCalibration.dir/src/positionCalibration.cpp.o.requires:

.PHONY : CMakeFiles/positionCalibration.dir/src/positionCalibration.cpp.o.requires

CMakeFiles/positionCalibration.dir/src/positionCalibration.cpp.o.provides: CMakeFiles/positionCalibration.dir/src/positionCalibration.cpp.o.requires
	$(MAKE) -f CMakeFiles/positionCalibration.dir/build.make CMakeFiles/positionCalibration.dir/src/positionCalibration.cpp.o.provides.build
.PHONY : CMakeFiles/positionCalibration.dir/src/positionCalibration.cpp.o.provides

CMakeFiles/positionCalibration.dir/src/positionCalibration.cpp.o.provides.build: CMakeFiles/positionCalibration.dir/src/positionCalibration.cpp.o


CMakeFiles/positionCalibration.dir/src/v4ldevice.cpp.o: CMakeFiles/positionCalibration.dir/flags.make
CMakeFiles/positionCalibration.dir/src/v4ldevice.cpp.o: src/v4ldevice.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/scanvandev/ScanVan/Calibration/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/positionCalibration.dir/src/v4ldevice.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/positionCalibration.dir/src/v4ldevice.cpp.o -c /home/scanvandev/ScanVan/Calibration/src/v4ldevice.cpp

CMakeFiles/positionCalibration.dir/src/v4ldevice.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/positionCalibration.dir/src/v4ldevice.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/scanvandev/ScanVan/Calibration/src/v4ldevice.cpp > CMakeFiles/positionCalibration.dir/src/v4ldevice.cpp.i

CMakeFiles/positionCalibration.dir/src/v4ldevice.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/positionCalibration.dir/src/v4ldevice.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/scanvandev/ScanVan/Calibration/src/v4ldevice.cpp -o CMakeFiles/positionCalibration.dir/src/v4ldevice.cpp.s

CMakeFiles/positionCalibration.dir/src/v4ldevice.cpp.o.requires:

.PHONY : CMakeFiles/positionCalibration.dir/src/v4ldevice.cpp.o.requires

CMakeFiles/positionCalibration.dir/src/v4ldevice.cpp.o.provides: CMakeFiles/positionCalibration.dir/src/v4ldevice.cpp.o.requires
	$(MAKE) -f CMakeFiles/positionCalibration.dir/build.make CMakeFiles/positionCalibration.dir/src/v4ldevice.cpp.o.provides.build
.PHONY : CMakeFiles/positionCalibration.dir/src/v4ldevice.cpp.o.provides

CMakeFiles/positionCalibration.dir/src/v4ldevice.cpp.o.provides.build: CMakeFiles/positionCalibration.dir/src/v4ldevice.cpp.o


# Object files for target positionCalibration
positionCalibration_OBJECTS = \
"CMakeFiles/positionCalibration.dir/src/positionCalibration.cpp.o" \
"CMakeFiles/positionCalibration.dir/src/v4ldevice.cpp.o"

# External object files for target positionCalibration
positionCalibration_EXTERNAL_OBJECTS =

bin/positionCalibration: CMakeFiles/positionCalibration.dir/src/positionCalibration.cpp.o
bin/positionCalibration: CMakeFiles/positionCalibration.dir/src/v4ldevice.cpp.o
bin/positionCalibration: CMakeFiles/positionCalibration.dir/build.make
bin/positionCalibration: /usr/local/lib/libopencv_stitching.so.3.4.0
bin/positionCalibration: /usr/local/lib/libopencv_superres.so.3.4.0
bin/positionCalibration: /usr/local/lib/libopencv_videostab.so.3.4.0
bin/positionCalibration: /usr/local/lib/libopencv_aruco.so.3.4.0
bin/positionCalibration: /usr/local/lib/libopencv_bgsegm.so.3.4.0
bin/positionCalibration: /usr/local/lib/libopencv_bioinspired.so.3.4.0
bin/positionCalibration: /usr/local/lib/libopencv_ccalib.so.3.4.0
bin/positionCalibration: /usr/local/lib/libopencv_cvv.so.3.4.0
bin/positionCalibration: /usr/local/lib/libopencv_dpm.so.3.4.0
bin/positionCalibration: /usr/local/lib/libopencv_face.so.3.4.0
bin/positionCalibration: /usr/local/lib/libopencv_freetype.so.3.4.0
bin/positionCalibration: /usr/local/lib/libopencv_fuzzy.so.3.4.0
bin/positionCalibration: /usr/local/lib/libopencv_img_hash.so.3.4.0
bin/positionCalibration: /usr/local/lib/libopencv_line_descriptor.so.3.4.0
bin/positionCalibration: /usr/local/lib/libopencv_optflow.so.3.4.0
bin/positionCalibration: /usr/local/lib/libopencv_reg.so.3.4.0
bin/positionCalibration: /usr/local/lib/libopencv_rgbd.so.3.4.0
bin/positionCalibration: /usr/local/lib/libopencv_saliency.so.3.4.0
bin/positionCalibration: /usr/local/lib/libopencv_stereo.so.3.4.0
bin/positionCalibration: /usr/local/lib/libopencv_structured_light.so.3.4.0
bin/positionCalibration: /usr/local/lib/libopencv_surface_matching.so.3.4.0
bin/positionCalibration: /usr/local/lib/libopencv_tracking.so.3.4.0
bin/positionCalibration: /usr/local/lib/libopencv_xfeatures2d.so.3.4.0
bin/positionCalibration: /usr/local/lib/libopencv_ximgproc.so.3.4.0
bin/positionCalibration: /usr/local/lib/libopencv_xobjdetect.so.3.4.0
bin/positionCalibration: /usr/local/lib/libopencv_xphoto.so.3.4.0
bin/positionCalibration: /usr/local/lib/libopencv_shape.so.3.4.0
bin/positionCalibration: /usr/local/lib/libopencv_photo.so.3.4.0
bin/positionCalibration: /usr/local/lib/libopencv_datasets.so.3.4.0
bin/positionCalibration: /usr/local/lib/libopencv_plot.so.3.4.0
bin/positionCalibration: /usr/local/lib/libopencv_text.so.3.4.0
bin/positionCalibration: /usr/local/lib/libopencv_dnn.so.3.4.0
bin/positionCalibration: /usr/local/lib/libopencv_ml.so.3.4.0
bin/positionCalibration: /usr/local/lib/libopencv_video.so.3.4.0
bin/positionCalibration: /usr/local/lib/libopencv_calib3d.so.3.4.0
bin/positionCalibration: /usr/local/lib/libopencv_features2d.so.3.4.0
bin/positionCalibration: /usr/local/lib/libopencv_highgui.so.3.4.0
bin/positionCalibration: /usr/local/lib/libopencv_videoio.so.3.4.0
bin/positionCalibration: /usr/local/lib/libopencv_phase_unwrapping.so.3.4.0
bin/positionCalibration: /usr/local/lib/libopencv_flann.so.3.4.0
bin/positionCalibration: /usr/local/lib/libopencv_imgcodecs.so.3.4.0
bin/positionCalibration: /usr/local/lib/libopencv_objdetect.so.3.4.0
bin/positionCalibration: /usr/local/lib/libopencv_imgproc.so.3.4.0
bin/positionCalibration: /usr/local/lib/libopencv_core.so.3.4.0
bin/positionCalibration: CMakeFiles/positionCalibration.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/scanvandev/ScanVan/Calibration/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Linking CXX executable bin/positionCalibration"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/positionCalibration.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/positionCalibration.dir/build: bin/positionCalibration

.PHONY : CMakeFiles/positionCalibration.dir/build

CMakeFiles/positionCalibration.dir/requires: CMakeFiles/positionCalibration.dir/src/positionCalibration.cpp.o.requires
CMakeFiles/positionCalibration.dir/requires: CMakeFiles/positionCalibration.dir/src/v4ldevice.cpp.o.requires

.PHONY : CMakeFiles/positionCalibration.dir/requires

CMakeFiles/positionCalibration.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/positionCalibration.dir/cmake_clean.cmake
.PHONY : CMakeFiles/positionCalibration.dir/clean

CMakeFiles/positionCalibration.dir/depend:
	cd /home/scanvandev/ScanVan/Calibration && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/scanvandev/ScanVan/Calibration /home/scanvandev/ScanVan/Calibration /home/scanvandev/ScanVan/Calibration /home/scanvandev/ScanVan/Calibration /home/scanvandev/ScanVan/Calibration/CMakeFiles/positionCalibration.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/positionCalibration.dir/depend

