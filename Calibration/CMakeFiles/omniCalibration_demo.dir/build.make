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

# Utility rule file for omniCalibration_demo.

# Include the progress variables for this target.
include CMakeFiles/omniCalibration_demo.dir/progress.make

CMakeFiles/omniCalibration_demo: bin/cmosCalibration
	bin/omniCalibration -w 9 -h 6 -sw 17.5 -sh 17.5 img/calib_dmk72buc02_omni/calib.xml

omniCalibration_demo: CMakeFiles/omniCalibration_demo
omniCalibration_demo: CMakeFiles/omniCalibration_demo.dir/build.make

.PHONY : omniCalibration_demo

# Rule to build all files generated by this target.
CMakeFiles/omniCalibration_demo.dir/build: omniCalibration_demo

.PHONY : CMakeFiles/omniCalibration_demo.dir/build

CMakeFiles/omniCalibration_demo.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/omniCalibration_demo.dir/cmake_clean.cmake
.PHONY : CMakeFiles/omniCalibration_demo.dir/clean

CMakeFiles/omniCalibration_demo.dir/depend:
	cd /home/scanvandev/ScanVan/Calibration && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/scanvandev/ScanVan/Calibration /home/scanvandev/ScanVan/Calibration /home/scanvandev/ScanVan/Calibration /home/scanvandev/ScanVan/Calibration /home/scanvandev/ScanVan/Calibration/CMakeFiles/omniCalibration_demo.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/omniCalibration_demo.dir/depend

