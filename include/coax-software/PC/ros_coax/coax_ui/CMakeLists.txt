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
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})

set(COAXHOME $ENV{HOME}/coax/coax-software/deploy)
set(COAXTARGET linux)
link_directories(${COAXHOME}/${COAXTARGET}/lib)
include_directories(${COAXHOME}/${COAXTARGET}/include)

# Coax GUI 

SET( COAXGUI_SRCS src/main.cpp src/CoaxGUIHandler.cpp)
SET( COAXGUI_MOC_HDRS src/CoaxGUIHandler.h)
# some .ui files
SET( COAXGUI_UIS src/CoaxGUI.ui)
# and finally an resource file
SET( COAXGUI_RCS CoaxGUI.qrc)

FIND_PACKAGE( Qt4 REQUIRED )
# this command will generate rules that will run rcc on all files from COAXGUI_RCS
# in result COAXGUI_RC_SRCS variable will contain paths to files produced by rcc
# QT4_ADD_RESOURCES( COAXGUI_RC_SRCS ${COAXGUI_RCS} )
INCLUDE( ${QT_USE_FILE} )
INCLUDE_DIRECTORIES( ${CMAKE_BINARY_DIR} )

# this will run uic on .ui files:
QT4_WRAP_UI( COAXGUI_UI_HDRS ${COAXGUI_UIS} )

# and finally this will run moc:
QT4_WRAP_CPP( COAXGUI_MOC_SRCS ${COAXGUI_MOC_HDRS} )


rosbuild_add_executable(CoaxGUI ${COAXGUI_SRCS} ${COAXGUI_MOC_SRCS} ${COAXGUI_RC_SRCS} ${COAXGUI_UI_HDRS} )
TARGET_LINK_LIBRARIES( CoaxGUI ${QT_LIBRARIES}  sbcom pthread rt)

