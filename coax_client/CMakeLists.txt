cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
rosbuild_genmsg()
#uncomment if you have defined services
rosbuild_gensrv()

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})

#rosbuild_add_executable(test_coax_liftoff src/TestCoaxAutoflight.cpp)
#rosbuild_add_executable(test_coax_idler src/TestCoaxIdler.cpp)
#rosbuild_add_executable(test_coax_client src/TestCoaxSubscriber.cpp)
#rosbuild_add_executable(test_coax_publisher src/CoaxVelocityControl.cpp)
rosbuild_add_executable(coax_filter_node src/CoaxFilteredStateNode.cpp)
rosbuild_add_executable(coax_localization_node src/CoaxLocalizationNode.cpp)
rosbuild_add_executable(coax_services_node src/CoaxServerNode.cpp)
#rosbuild_add_executable(test_coax_distance src/CoaxDistanceCalc.cpp)
#rosbuild_add_executable(test_matrix src/test_matrix.cpp)
