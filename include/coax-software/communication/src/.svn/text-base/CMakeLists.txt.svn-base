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

ADD_LIBRARY(
	sbcom
	STATIC
	sbapi.c
	sbsimple.c
	sbmessage.c
	sbstate.c
	sbchannel.c
	sbchannel_serial.c
	sbchannel_socket.c
	sbcommloop.c
	sbversion.c
)

INSTALL_TARGETS(/lib sbcom)
INSTALL_FILES(/include/com .h 
	../include/com/sbversion.h
	../include/com/sbapi.h
	../include/com/sbchannel.h
	../include/com/sbconst.h
	../include/com/sbmessage.h
	../include/com/sbsimple.h
	../include/com/sbstate.h
	../include/com/sbcommloop.h
)
