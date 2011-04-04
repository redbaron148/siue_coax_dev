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
SET(CROSSCOMPILING yes)
SET( CROSS_SYSTEM arm-linux )
SET( BIN_PREFIX arm-angstrom-linux-gnueabi )

SET(GUM_ROOTPATH ${HOME}/home/cedricp/coax/overo-oe)
SET(GUM_GCC ${GUM_ROOTPATH}/tmp/cross/armv7a/bin/arm-angstrom-linux-gnueabi-gcc-sysroot)
SET(GUM_GXX ${GUM_ROOTPATH}/tmp/cross/armv7a/bin/arm-angstrom-linux-gnueabi-g++-sysroot)

MESSAGE( STATUS "Found compiler: ${GUM_GCC}" )
MESSAGE( STATUS "Found compiler: ${GUM_GXX}" )


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
# SET( CMAKE_INSTALL_PREFIX ${GUM_ROOTPATH}/build_arm_nofpu/root/usr )
SET(CMAKE_INSTALL_PREFIX "${CMAKE_HOME_DIRECTORY}/../deploy/gumstix")

# Set module search path
SET( CMAKE_MODULE_PATH ${CMAKE_INSTALL_PREFIX}/lib/modules )

	

