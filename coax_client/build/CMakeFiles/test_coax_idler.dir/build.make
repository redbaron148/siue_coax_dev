# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 2.6

#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canoncical targets will work.
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

# The program to use to edit the cache.
CMAKE_EDIT_COMMAND = /usr/bin/ccmake

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/ros-vm/ros/stacks/coax_client

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/ros-vm/ros/stacks/coax_client/build

# Include any dependencies generated for this target.
include CMakeFiles/test_coax_idler.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/test_coax_idler.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/test_coax_idler.dir/flags.make

CMakeFiles/test_coax_idler.dir/src/TestCoaxIdler.o: CMakeFiles/test_coax_idler.dir/flags.make
CMakeFiles/test_coax_idler.dir/src/TestCoaxIdler.o: ../src/TestCoaxIdler.cpp
CMakeFiles/test_coax_idler.dir/src/TestCoaxIdler.o: ../manifest.xml
CMakeFiles/test_coax_idler.dir/src/TestCoaxIdler.o: /home/ros-vm/ros/ros/core/roslang/manifest.xml
CMakeFiles/test_coax_idler.dir/src/TestCoaxIdler.o: /home/ros-vm/ros/ros/core/genmsg_cpp/manifest.xml
CMakeFiles/test_coax_idler.dir/src/TestCoaxIdler.o: /home/ros-vm/ros/ros/tools/rospack/manifest.xml
CMakeFiles/test_coax_idler.dir/src/TestCoaxIdler.o: /home/ros-vm/ros/ros/core/roslib/manifest.xml
CMakeFiles/test_coax_idler.dir/src/TestCoaxIdler.o: /home/ros-vm/ros/ros/3rdparty/xmlrpcpp/manifest.xml
CMakeFiles/test_coax_idler.dir/src/TestCoaxIdler.o: /home/ros-vm/ros/ros/core/rosconsole/manifest.xml
CMakeFiles/test_coax_idler.dir/src/TestCoaxIdler.o: /home/ros-vm/ros/ros/core/roscpp/manifest.xml
CMakeFiles/test_coax_idler.dir/src/TestCoaxIdler.o: /home/ros-vm/ros/ros/core/rospy/manifest.xml
CMakeFiles/test_coax_idler.dir/src/TestCoaxIdler.o: /home/ros-vm/ros/stacks/ros_coax/coax_msgs/manifest.xml
	$(CMAKE_COMMAND) -E cmake_progress_report /home/ros-vm/ros/stacks/coax_client/build/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/test_coax_idler.dir/src/TestCoaxIdler.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -W -Wall -Wno-unused-parameter -fno-strict-aliasing -pthread -o CMakeFiles/test_coax_idler.dir/src/TestCoaxIdler.o -c /home/ros-vm/ros/stacks/coax_client/src/TestCoaxIdler.cpp

CMakeFiles/test_coax_idler.dir/src/TestCoaxIdler.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/test_coax_idler.dir/src/TestCoaxIdler.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -W -Wall -Wno-unused-parameter -fno-strict-aliasing -pthread -E /home/ros-vm/ros/stacks/coax_client/src/TestCoaxIdler.cpp > CMakeFiles/test_coax_idler.dir/src/TestCoaxIdler.i

CMakeFiles/test_coax_idler.dir/src/TestCoaxIdler.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/test_coax_idler.dir/src/TestCoaxIdler.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -W -Wall -Wno-unused-parameter -fno-strict-aliasing -pthread -S /home/ros-vm/ros/stacks/coax_client/src/TestCoaxIdler.cpp -o CMakeFiles/test_coax_idler.dir/src/TestCoaxIdler.s

CMakeFiles/test_coax_idler.dir/src/TestCoaxIdler.o.requires:
.PHONY : CMakeFiles/test_coax_idler.dir/src/TestCoaxIdler.o.requires

CMakeFiles/test_coax_idler.dir/src/TestCoaxIdler.o.provides: CMakeFiles/test_coax_idler.dir/src/TestCoaxIdler.o.requires
	$(MAKE) -f CMakeFiles/test_coax_idler.dir/build.make CMakeFiles/test_coax_idler.dir/src/TestCoaxIdler.o.provides.build
.PHONY : CMakeFiles/test_coax_idler.dir/src/TestCoaxIdler.o.provides

CMakeFiles/test_coax_idler.dir/src/TestCoaxIdler.o.provides.build: CMakeFiles/test_coax_idler.dir/src/TestCoaxIdler.o
.PHONY : CMakeFiles/test_coax_idler.dir/src/TestCoaxIdler.o.provides.build

# Object files for target test_coax_idler
test_coax_idler_OBJECTS = \
"CMakeFiles/test_coax_idler.dir/src/TestCoaxIdler.o"

# External object files for target test_coax_idler
test_coax_idler_EXTERNAL_OBJECTS =

../bin/test_coax_idler: CMakeFiles/test_coax_idler.dir/src/TestCoaxIdler.o
../bin/test_coax_idler: CMakeFiles/test_coax_idler.dir/build.make
../bin/test_coax_idler: CMakeFiles/test_coax_idler.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX executable ../bin/test_coax_idler"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/test_coax_idler.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/test_coax_idler.dir/build: ../bin/test_coax_idler
.PHONY : CMakeFiles/test_coax_idler.dir/build

CMakeFiles/test_coax_idler.dir/requires: CMakeFiles/test_coax_idler.dir/src/TestCoaxIdler.o.requires
.PHONY : CMakeFiles/test_coax_idler.dir/requires

CMakeFiles/test_coax_idler.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/test_coax_idler.dir/cmake_clean.cmake
.PHONY : CMakeFiles/test_coax_idler.dir/clean

CMakeFiles/test_coax_idler.dir/depend:
	cd /home/ros-vm/ros/stacks/coax_client/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/ros-vm/ros/stacks/coax_client /home/ros-vm/ros/stacks/coax_client /home/ros-vm/ros/stacks/coax_client/build /home/ros-vm/ros/stacks/coax_client/build /home/ros-vm/ros/stacks/coax_client/build/CMakeFiles/test_coax_idler.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/test_coax_idler.dir/depend

