##############################################################################################
# Author: Christoph Hürzeler Date: 11.11.08
# Cross-compilation setup for gumstix
#
# This file is passed to cmake on the command line via
# -DCMAKE_TOOLCHAIN_FILE.  It gets read first, prior to any of cmake's
# system tests.
# Alternatively INCLUDE(gum.cmake) maybe used at the top level CMakeLists.txt
#TODO: Find a better CMake compatible shell command than locate to find files
#      find ${HOME} -name "something" -printf %p would be an option which takes longer than
#      locate and runs into trouble when more than one pattern match occurs
##############################################################################################

# Not acutally required could copy directly to CMAKE_SYSTEM_NAME ...
SET( CROSS_SYSTEM arm-linux )

# Check if /usr/bin/locate is available
#TODO: Make this cross-platform compatible (does not make too much sense for gumstix...)
#FIND_PROGRAM( LX_LOCATE NAMES locate )


# Search HOME for "gumstix-buildroot" directory. First found directory is taken.
# Might need to "updatedb"...
# NOTE: Unfortunately FIND_PATH does not allow searching recursively
MESSAGE( STATUS "Searching gumstix-buildroot directory..." )
EXECUTE_PROCESS( COMMAND find $ENV{HOME} -type d -name "*gumstix-buildroot"
		 COMMAND head -n 1
		 COMMAND tr -d '\n'
		 OUTPUT_VARIABLE GUM_ROOTPATH )

IF( NOT GUM_ROOTPATH )
  MESSAGE( FATAL_ERROR "gumstix-buildroot directory not found. Aborting." )
ENDIF( NOT GUM_ROOTPATH )

MESSAGE( STATUS "Found directory: ${GUM_ROOTPATH}" )
MESSAGE( STATUS "" )

# Search GUM_ROOTPATH for arm-linux-gcc
MESSAGE( STATUS "Searching your arm-linux-gcc compiler..." )
EXECUTE_PROCESS( COMMAND find ${GUM_ROOTPATH} -name "arm-linux-gcc"
		 COMMAND head -n 1
		 COMMAND tr -d '\n'
		 OUTPUT_VARIABLE GUM_GCC )

IF( NOT GUM_GCC )
  MESSAGE( FATAL_ERROR "arm-linux-gcc compiler not found. Aborting." )
ENDIF( NOT GUM_GCC )

MESSAGE( STATUS "Found compiler: ${GUM_GCC}" )
MESSAGE( STATUS "" )


# Search GUM_ROOTPATH for arm-linux-g++
MESSAGE( STATUS "Searching your arm-linux-g++ compiler..." )
EXECUTE_PROCESS( COMMAND find ${GUM_ROOTPATH} -name "arm-linux-g++"
		 COMMAND head -n 1
		 COMMAND tr -d '\n'
		 OUTPUT_VARIABLE GUM_GXX )

IF( NOT GUM_GXX )
  MESSAGE( FATAL_ERROR "arm-linux-g++ compiler not found. Aborting." )
ENDIF( NOT GUM_GXX )

MESSAGE( STATUS "Found compiler: ${GUM_GXX}" )
MESSAGE( STATUS "" )

#TODO: Search for ar, strip and ranlib?

# Setup cross-compiling
SET( CMAKE_SYSTEM_NAME  ${CROSS_SYSTEM} )
SET( CMAKE_C_COMPILER   ${GUM_GCC} )
SET( CMAKE_CXX_COMPILER ${GUM_GXX} )
SET( CMAKE_FIND_ROOT_PATH ${GUM_ROOTPATH}/build_arm_nofpu/root )
SET( CMAKE_FIND_ROOT_PATH_MODE_PROGRAM BOTH )# set to only if only gum-root shall be searched
SET( CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY )
SET( CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY )

# Install cross-compiled libraries to root of local gumstix environment
SET( CMAKE_INSTALL_PREFIX ${GUM_ROOTPATH}/build_arm_nofpu/root/usr )

# Set module search path
SET( CMAKE_MODULE_PATH ${CMAKE_INSTALL_PREFIX}/lib/modules )
