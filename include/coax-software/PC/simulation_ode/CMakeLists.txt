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
# 

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
# 
#
# CMakeList to compile the simulation stuff
#
cmake_minimum_required(VERSION 2.4)

#Name of the Project
PROJECT(coaxSim)

FIND_LIBRARY(SYSLIBS pthread rt m)
SET(CMAKE_INSTALL_PREFIX "${CMAKE_HOME_DIRECTORY}/../../deploy/linux")

FIND_PROGRAM(ODECONFIG_EXECUTABLE NAMES ode-config 
	PATHS /usr/bin /usr/local/bin ${CMAKE_INSTALL_PREFIX}/bin)

EXEC_PROGRAM(${ODECONFIG_EXECUTABLE} ARGS "--cflags" OUTPUT_VARIABLE _odeconfigOutput)

SET(CMAKE_C_FLAGS "-std=c99")
ADD_DEFINITIONS(-Wall -ggdb -g -DDRAWSTUFF_TEXTURE_PATH=\\"../textures\\"
		       -DdTRIMESH_ENABLED ${_odeconfigOutput}  -g 
			   -DLINUX -DSBC_HAS_PTHREAD -DSBC_HAS_COMM -DSBC_HAS_IO
			   -DSIMULATION -DDEFENSIVE
)

INCLUDE_DIRECTORIES(${CMAKE_HOME_DIRECTORY} ${CMAKE_INSTALL_PREFIX}/include)
LINK_DIRECTORIES(${CMAKE_INSTALL_PREFIX}/lib /usr/local/lib)




SET(SRCS coaxsim.cpp PID.cpp V3.cpp M3.cpp)

ADD_LIBRARY(drawstuff STATIC
	drawstuff/JpegCodec.cpp
	drawstuff/drawstuff.cpp
	#drawstuff/osx.cpp
	#drawstuff/windows.cpp
	drawstuff/x11.cpp
)


ADD_EXECUTABLE(coaxSim ${SRCS})
#ADD_EXECUTABLE(coaxSim coaxSim.cpp)


TARGET_LINK_LIBRARIES(coaxSim 
	sbcom pthread rt
	drawstuff ode SM ICE GL GLU jpeg X11 m) 

INSTALL_TARGETS(/bin coaxSim)
