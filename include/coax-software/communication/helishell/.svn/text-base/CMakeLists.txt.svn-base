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

INCLUDE_DIRECTORIES(../src)
LINK_DIRECTORIES(../lib)


# ADD_EXECUTABLE(
# 	helishell
# 	helishell.c
# )
# 
# TARGET_LINK_LIBRARIES( helishell sbcom pthread rt m)
# 	
# INSTALL_TARGETS(/bin helishell)

ADD_EXECUTABLE(
	simplehelishell
	simplehelishell.cpp
	MenuManager.cpp
)

TARGET_LINK_LIBRARIES( simplehelishell sbcom pthread ${READLINE_LIB} rt m)
	
INSTALL_TARGETS(/bin simplehelishell)
