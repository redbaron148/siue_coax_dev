# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 2.8

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

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/aaron/ros_pkgs/siue_coax_dev/ros_coax_dev/coax_server

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/aaron/ros_pkgs/siue_coax_dev/ros_coax_dev/coax_server/build

# Include any dependencies generated for this target.
include CMakeFiles/coax_server.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/coax_server.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/coax_server.dir/flags.make

CMakeFiles/coax_server.dir/src/coax_server.o: CMakeFiles/coax_server.dir/flags.make
CMakeFiles/coax_server.dir/src/coax_server.o: ../src/coax_server.cpp
CMakeFiles/coax_server.dir/src/coax_server.o: ../manifest.xml
CMakeFiles/coax_server.dir/src/coax_server.o: /opt/ros/cturtle/ros/core/genmsg_cpp/manifest.xml
CMakeFiles/coax_server.dir/src/coax_server.o: /opt/ros/cturtle/ros/tools/rospack/manifest.xml
CMakeFiles/coax_server.dir/src/coax_server.o: /opt/ros/cturtle/ros/core/roslib/manifest.xml
CMakeFiles/coax_server.dir/src/coax_server.o: /opt/ros/cturtle/ros/core/rosconsole/manifest.xml
CMakeFiles/coax_server.dir/src/coax_server.o: /opt/ros/cturtle/ros/std_msgs/manifest.xml
CMakeFiles/coax_server.dir/src/coax_server.o: /opt/ros/cturtle/ros/core/roslang/manifest.xml
CMakeFiles/coax_server.dir/src/coax_server.o: /opt/ros/cturtle/ros/core/rospy/manifest.xml
CMakeFiles/coax_server.dir/src/coax_server.o: /opt/ros/cturtle/ros/3rdparty/xmlrpcpp/manifest.xml
CMakeFiles/coax_server.dir/src/coax_server.o: /opt/ros/cturtle/ros/core/roscpp/manifest.xml
CMakeFiles/coax_server.dir/src/coax_server.o: /home/aaron/ros_pkgs/siue_coax_dev/ros_coax_dev/coax_msgs/manifest.xml
CMakeFiles/coax_server.dir/src/coax_server.o: /opt/ros/cturtle/ros/core/roslib/msg_gen/generated
CMakeFiles/coax_server.dir/src/coax_server.o: /opt/ros/cturtle/ros/std_msgs/msg_gen/generated
CMakeFiles/coax_server.dir/src/coax_server.o: /opt/ros/cturtle/ros/core/roscpp/msg_gen/generated
CMakeFiles/coax_server.dir/src/coax_server.o: /opt/ros/cturtle/ros/core/roscpp/srv_gen/generated
CMakeFiles/coax_server.dir/src/coax_server.o: /home/aaron/ros_pkgs/siue_coax_dev/ros_coax_dev/coax_msgs/msg_gen/generated
CMakeFiles/coax_server.dir/src/coax_server.o: /home/aaron/ros_pkgs/siue_coax_dev/ros_coax_dev/coax_msgs/srv_gen/generated
	$(CMAKE_COMMAND) -E cmake_progress_report /home/aaron/ros_pkgs/siue_coax_dev/ros_coax_dev/coax_server/build/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/coax_server.dir/src/coax_server.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -W -Wall -Wno-unused-parameter -fno-strict-aliasing -pthread -o CMakeFiles/coax_server.dir/src/coax_server.o -c /home/aaron/ros_pkgs/siue_coax_dev/ros_coax_dev/coax_server/src/coax_server.cpp

CMakeFiles/coax_server.dir/src/coax_server.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/coax_server.dir/src/coax_server.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -W -Wall -Wno-unused-parameter -fno-strict-aliasing -pthread -E /home/aaron/ros_pkgs/siue_coax_dev/ros_coax_dev/coax_server/src/coax_server.cpp > CMakeFiles/coax_server.dir/src/coax_server.i

CMakeFiles/coax_server.dir/src/coax_server.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/coax_server.dir/src/coax_server.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -W -Wall -Wno-unused-parameter -fno-strict-aliasing -pthread -S /home/aaron/ros_pkgs/siue_coax_dev/ros_coax_dev/coax_server/src/coax_server.cpp -o CMakeFiles/coax_server.dir/src/coax_server.s

CMakeFiles/coax_server.dir/src/coax_server.o.requires:
.PHONY : CMakeFiles/coax_server.dir/src/coax_server.o.requires

CMakeFiles/coax_server.dir/src/coax_server.o.provides: CMakeFiles/coax_server.dir/src/coax_server.o.requires
	$(MAKE) -f CMakeFiles/coax_server.dir/build.make CMakeFiles/coax_server.dir/src/coax_server.o.provides.build
.PHONY : CMakeFiles/coax_server.dir/src/coax_server.o.provides

CMakeFiles/coax_server.dir/src/coax_server.o.provides.build: CMakeFiles/coax_server.dir/src/coax_server.o
.PHONY : CMakeFiles/coax_server.dir/src/coax_server.o.provides.build

# Object files for target coax_server
coax_server_OBJECTS = \
"CMakeFiles/coax_server.dir/src/coax_server.o"

# External object files for target coax_server
coax_server_EXTERNAL_OBJECTS =

../bin/coax_server: CMakeFiles/coax_server.dir/src/coax_server.o
../bin/coax_server: CMakeFiles/coax_server.dir/build.make
../bin/coax_server: CMakeFiles/coax_server.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX executable ../bin/coax_server"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/coax_server.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/coax_server.dir/build: ../bin/coax_server
.PHONY : CMakeFiles/coax_server.dir/build

CMakeFiles/coax_server.dir/requires: CMakeFiles/coax_server.dir/src/coax_server.o.requires
.PHONY : CMakeFiles/coax_server.dir/requires

CMakeFiles/coax_server.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/coax_server.dir/cmake_clean.cmake
.PHONY : CMakeFiles/coax_server.dir/clean

CMakeFiles/coax_server.dir/depend:
	cd /home/aaron/ros_pkgs/siue_coax_dev/ros_coax_dev/coax_server/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/aaron/ros_pkgs/siue_coax_dev/ros_coax_dev/coax_server /home/aaron/ros_pkgs/siue_coax_dev/ros_coax_dev/coax_server /home/aaron/ros_pkgs/siue_coax_dev/ros_coax_dev/coax_server/build /home/aaron/ros_pkgs/siue_coax_dev/ros_coax_dev/coax_server/build /home/aaron/ros_pkgs/siue_coax_dev/ros_coax_dev/coax_server/build/CMakeFiles/coax_server.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/coax_server.dir/depend

