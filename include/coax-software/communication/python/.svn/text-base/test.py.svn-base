#!/usr/bin/python2.5
# *************************************************************
# 
#  API and Communication library for the Skybotix AG
#  helicopters, and particularly COAX
# 
#  Developed by Cedric Pradalier: cedric.pradalier@skybotix.ch
#  Send modification or corrections as patches (diff -Naur)
#  Copyright: Skybotix AG, 2009-2012
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
# *************************************************************

import sys
from math import *
from skybotix.sbsimple import *

print "Creating conn object"
conn = sbSimpleAlloc('localhost',5123);
print "Starting initialisation"

if sbSimpleInitialise(conn) != 0:
	print "Error in initialisation"
	sbSimpleFree(conn)
	sys.exit(-1)
	

print "Reaching Ctrlled state"
# request and wait controlled mode
sbSimpleReachNavState(conn, SB_NAV_CTRLLED, 30.0);
print 'Heli is controlled'

sbSimpleWaitState(conn,None,1.0);

print 'Checking take-off (%f)' % conn.state.zrange
while conn.state.zrange < 0.4:
	sbSimpleControl(conn,0,0,0, 0.5);
	sbSimpleWaitState(conn,None,1.0);

print 'Rotation'
for i in range(1,100):
	sbSimpleControl(conn,0,0,sin(2*pi*i/100)*2.0, 0.5);
	sbSimpleWaitState(conn,None,1.0);
	print "%+3f" % (conn.state.yaw * 180/pi)

print 'Pitch'
for i in range(1,100):
	sbSimpleControl(conn,0,sin(2*pi*i/100)*0.01, 0, 0.5);
	sbSimpleWaitState(conn,None,1.0);

print 'Roll'
for i in range(1,100):
	sbSimpleControl(conn,sin(2*pi*i/100)*0.01, 0, 0, 0.5);
	sbSimpleWaitState(conn,None,1.0);

sbSimpleReachNavState(conn, SB_NAV_STOP, 30.0);

print 'Cleanup'
sbSimpleTerminate(conn);
sbSimpleFree(conn)

