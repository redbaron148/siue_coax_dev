# 
# All rights reserved.
# 
# Skybotix API is free software: you can redistribute it and/or modify
# it under the terms of the GNU Lesser General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
# 
# Skybotix API is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
# 
# You should have received a copy of the GNU Lesser General Public License
# along with Skybotix API. If not, see <http://www.gnu.org/licenses/>.


CMAKE_MINIMUM_REQUIRED(VERSION 2.4.6)
MARK_AS_ADVANCED( CMAKE_BACKWARDS_COMPATIBILITY )

# Define project
PROJECT( coax-communication )
MESSAGE( STATUS "Configuring project ${PROJECT_NAME}" )

# lib-version 0.1.0
SET( ${PROJECT_NAME}_MAJOR_VERSION 0 )
SET( ${PROJECT_NAME}_MINOR_VERSION 1 )
SET( ${PROJECT_NAME}_PATCH_LEVEL   0 )

# Project options
#OPTION( BUILD_SHARED_LIBS "Set to OFF to build static libraries" ON )
#OPTION( INSTALL_DOC "Set to OFF to skip build/install Documentation" ON )
OPTION( CROSSCOMPILING "Set to ON for cross compiling" OFF )


SET(CMAKE_INSTALL_PREFIX "${CMAKE_HOME_DIRECTORY}/../deploy/linux")

## Check if cross compiling
IF( CROSSCOMPILING )
	#INCLUDE( ${CMAKE_HOME_DIRECTORY}/modules/gum.cmake )
  INCLUDE( ${CMAKE_HOME_DIRECTORY}/modules/overo.cmake )
ENDIF( CROSSCOMPILING )

FIND_LIBRARY(SYSLIBS pthread rt m)
LINK_DIRECTORIES(${CMAKE_BINARY_DIR}/src)


# Print build information for user
#MESSAGE(STATUS "Project dependencies: \"${PROJECT_NAME}_DEPENDS\" ")
#MESSAGE( STATUS "Compiling shared libraries: ${BUILD_SHARED_LIBS} " )
#MESSAGE( STATUS "Installing documentation: ${INSTALL_DOC}" )
MESSAGE( STATUS "" )
MESSAGE( STATUS "C-compiler:   \"${CMAKE_C_COMPILER}\" " )
MESSAGE( STATUS "C++-compiler: \"${CMAKE_CXX_COMPILER}\" " )
MESSAGE( STATUS "Cross compiling: ${CROSSCOMPILING}" )
MESSAGE( STATUS "" )
#MESSAGE( STATUS "Library '${PROJECT_NAME}' will be installed in ${CMAKE_INSTALL_PREFIX}" )
#MESSAGE( STATUS "" )
MESSAGE( STATUS "   When you're satisfied with the configuration, type 'make install' " )
MESSAGE( STATUS "" )


IF( NOT CROSSCOMPILING )
MESSAGE( STATUS "" )
  MESSAGE( STATUS "Type 'cmake . -DCROSSCOMPILING=yes' to cross-compile the project" )
  MESSAGE( STATUS "" )
ENDIF( NOT CROSSCOMPILING )







IF( CROSSCOMPILING )
	SET( USE_READLINE no)
ELSE( CROSSCOMPILING )
	SET( USE_READLINE yes)
ENDIF( CROSSCOMPILING )

IF( USE_READLINE)
	SET( READLINE_DEF -DUSE_READLINE )
	SET( READLINE_LIB -lreadline )
ENDIF( USE_READLINE)

INCLUDE_DIRECTORIES(${CMAKE_HOME_DIRECTORY}/include)

ADD_DEFINITIONS( -ggdb -Wall -D_GNU_SOURCE -DSBC_HAS_IO -DDEFENSIVE -DSBC_HAS_COMM
	-DSBC_HAS_PTHREAD -DLINUX ${READLINE_DEF} -DSIMULATION)
#ADD_DEFINITIONS(-DUSE_SOCKET)
#ADD_DEFINITIONS( -ggdb -march=pentium4 -O3 -pipe -Wall )


SUBDIRS(
	src 
	examples
	repeater
	helishell
	# bootloader
)
	

