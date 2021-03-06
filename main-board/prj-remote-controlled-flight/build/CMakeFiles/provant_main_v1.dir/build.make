# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 2.8

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
CMAKE_SOURCE_DIR = /home/fernando/workspace/provant-software_mutex/main-board/prj-remote-controlled-flight

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/fernando/workspace/provant-software_mutex/main-board/prj-remote-controlled-flight/build

# Include any dependencies generated for this target.
include CMakeFiles/provant_main_v1.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/provant_main_v1.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/provant_main_v1.dir/flags.make

CMakeFiles/provant_main_v1.dir/main/src/main.cpp.o: CMakeFiles/provant_main_v1.dir/flags.make
CMakeFiles/provant_main_v1.dir/main/src/main.cpp.o: ../main/src/main.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/fernando/workspace/provant-software_mutex/main-board/prj-remote-controlled-flight/build/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/provant_main_v1.dir/main/src/main.cpp.o"
	arm-linux-gnueabihf-g++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/provant_main_v1.dir/main/src/main.cpp.o -c /home/fernando/workspace/provant-software_mutex/main-board/prj-remote-controlled-flight/main/src/main.cpp

CMakeFiles/provant_main_v1.dir/main/src/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/provant_main_v1.dir/main/src/main.cpp.i"
	arm-linux-gnueabihf-g++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/fernando/workspace/provant-software_mutex/main-board/prj-remote-controlled-flight/main/src/main.cpp > CMakeFiles/provant_main_v1.dir/main/src/main.cpp.i

CMakeFiles/provant_main_v1.dir/main/src/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/provant_main_v1.dir/main/src/main.cpp.s"
	arm-linux-gnueabihf-g++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/fernando/workspace/provant-software_mutex/main-board/prj-remote-controlled-flight/main/src/main.cpp -o CMakeFiles/provant_main_v1.dir/main/src/main.cpp.s

CMakeFiles/provant_main_v1.dir/main/src/main.cpp.o.requires:
.PHONY : CMakeFiles/provant_main_v1.dir/main/src/main.cpp.o.requires

CMakeFiles/provant_main_v1.dir/main/src/main.cpp.o.provides: CMakeFiles/provant_main_v1.dir/main/src/main.cpp.o.requires
	$(MAKE) -f CMakeFiles/provant_main_v1.dir/build.make CMakeFiles/provant_main_v1.dir/main/src/main.cpp.o.provides.build
.PHONY : CMakeFiles/provant_main_v1.dir/main/src/main.cpp.o.provides

CMakeFiles/provant_main_v1.dir/main/src/main.cpp.o.provides.build: CMakeFiles/provant_main_v1.dir/main/src/main.cpp.o

# Object files for target provant_main_v1
provant_main_v1_OBJECTS = \
"CMakeFiles/provant_main_v1.dir/main/src/main.cpp.o"

# External object files for target provant_main_v1
provant_main_v1_EXTERNAL_OBJECTS =

provant_main_v1: CMakeFiles/provant_main_v1.dir/main/src/main.cpp.o
provant_main_v1: CMakeFiles/provant_main_v1.dir/build.make
provant_main_v1: ../subsystems/subsys_DataProcessing/build/libsubsys_DataProcessing_bundle.a
provant_main_v1: ../subsystems/subsys_ContinuousControl/build/libsubsys_ContinuousControl_bundle.a
provant_main_v1: ../subsystems/subsys_CommLowLevel/build/libsubsys_CommLowLevel_bundle.a
provant_main_v1: CMakeFiles/provant_main_v1.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX executable provant_main_v1"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/provant_main_v1.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/provant_main_v1.dir/build: provant_main_v1
.PHONY : CMakeFiles/provant_main_v1.dir/build

CMakeFiles/provant_main_v1.dir/requires: CMakeFiles/provant_main_v1.dir/main/src/main.cpp.o.requires
.PHONY : CMakeFiles/provant_main_v1.dir/requires

CMakeFiles/provant_main_v1.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/provant_main_v1.dir/cmake_clean.cmake
.PHONY : CMakeFiles/provant_main_v1.dir/clean

CMakeFiles/provant_main_v1.dir/depend:
	cd /home/fernando/workspace/provant-software_mutex/main-board/prj-remote-controlled-flight/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/fernando/workspace/provant-software_mutex/main-board/prj-remote-controlled-flight /home/fernando/workspace/provant-software_mutex/main-board/prj-remote-controlled-flight /home/fernando/workspace/provant-software_mutex/main-board/prj-remote-controlled-flight/build /home/fernando/workspace/provant-software_mutex/main-board/prj-remote-controlled-flight/build /home/fernando/workspace/provant-software_mutex/main-board/prj-remote-controlled-flight/build/CMakeFiles/provant_main_v1.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/provant_main_v1.dir/depend

