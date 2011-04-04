#!/usr/bin/python
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


import time
import traceback
import os
import signal
import re
import math
import BaseHTTPServer
import threading
import SimpleHTTPServer

from skybotix.sbsimple import *

coax_device = "/dev/ttyS0"
coax_port = 5123
repeater_cmd = "../build-gumstix/repeater/repeater /dev/ttyS0 -5123"
video_cmd = "ping localhost"
repeater_pid = 0
video_pid = 0
heliconn = None
processingReq = False
repeater = None

def create_content_string(content):
	output = "%08lX [" % content
	if (content & SBS_MODES):
		output += "%s," % (sbContentString(SBS_MODES))
	if (content & SBS_TIMESTAMP):
		output += "%s," % (sbContentString(SBS_TIMESTAMP))
	if (content & SBS_RPY):
		output += "%s," % (sbContentString(SBS_RPY))
	if (content & SBS_GYRO):
		output += "%s," % (sbContentString(SBS_GYRO))
	if (content & SBS_ACCEL):
		output += "%s," % (sbContentString(SBS_ACCEL))
	if (content & SBS_MAGNETO):
		output += "%s," % (sbContentString(SBS_MAGNETO))
	if (content & SBS_IMUTEMP):
		output += "%s," % (sbContentString(SBS_IMUTEMP))
	if (content & SBS_ZRANGE):
		output += "%s," % (sbContentString(SBS_ZRANGE))
	if (content & SBS_ZPRESSURE):
		output += "%s," % (sbContentString(SBS_ZPRESSURE))
	if (content & SBS_HRANGES):
		output += "%s," % (sbContentString(SBS_HRANGES))
	if (content & SBS_XY_REL):
		output += "%s," % (sbContentString(SBS_XY_REL))
	if (content & SBS_BATTERY):
		output += "%s," % (sbContentString(SBS_BATTERY))
	if (content & SBS_TIMEOUT):
		output += "%s," % (sbContentString(SBS_TIMEOUT))
	if (content & SBS_O_ATTITUDE):
		output += "%s," % (sbContentString(SBS_O_ATTITUDE))
	if (content & SBS_O_ALTITUDE):
		output += "%s," % (sbContentString(SBS_O_ALTITUDE))
	if (content & SBS_O_TOL):
		output += "%s," % (sbContentString(SBS_O_TOL))
	if (content & SBS_O_XY):
		output += "%s," % (sbContentString(SBS_O_XY))
	if (content & SBS_O_OAVOID):
		output += "%s," % (sbContentString(SBS_O_OAVOID))
	output += "]"
	return output

def create_status_gen(state,startlist,endlist, prefix, suffix):
	def R2D(x):
		return x * 180./math.pi
	output = startlist
	output += prefix + "Error  : %02X" % (state.errorFlags) + suffix
	output += prefix + "Content: " + \
			create_content_string(state.content) + suffix
	if (state.content & SBS_MODES):
		output += prefix + "Modes:" + startlist
		output += prefix + "Navigation %s" % sbNavModeString(state.mode.navigation) + suffix
		output += prefix + "Communication %s" % sbCommModeString(state.mode.communication) + suffix
		output += prefix + "Obst. Avoid: %s" % sbOAModeString(state.mode.oavoid) + suffix
		output += prefix + "Axis: Roll %s" % sbCtrlModeString(state.mode.rollAxis) + suffix
		output += prefix + "Pitch %s" % sbCtrlModeString(state.mode.pitchAxis) + suffix
		output += prefix + "Yaw %s" % sbCtrlModeString(state.mode.yawAxis) + suffix
		output += prefix + "Alti. %s" % sbCtrlModeString(state.mode.altAxis) + suffix
		output += endlist 
	if (state.content & SBS_TIMESTAMP):
		output += prefix + "Timestamp: %ld" % (state.timeStamp) + suffix
	if (state.content & SBS_RPY):
		output += prefix + "RPY (deg): %.2f, %.2f, %.2f" % (R2D(state.roll), R2D(state.pitch), R2D(state.yaw)) + suffix
	if (state.content & SBS_GYRO):
		output += prefix + "Gyro (deg/s): %.2f, %.2f, %.2f" % \
				(R2D(sbGetFloat(state.gyro,0)), \
				R2D(sbGetFloat(state.gyro,1)), \
				R2D(sbGetFloat(state.gyro,2))) + suffix
	if (state.content & SBS_ACCEL):
		output += prefix + "Accel (m/s2): %.2f, %.2f, %.2f" % (sbGetFloat(state.accel,0), sbGetFloat(state.accel,1), sbGetFloat(state.accel,2)) + suffix
	if (state.content & SBS_MAGNETO):
		output += prefix + "Magneto: %.2f, %.2f, %.2f" % (sbGetFloat(state.magneto,0), sbGetFloat(state.magneto,1), sbGetFloat(state.magneto,2)) + suffix
	if (state.content & SBS_IMUTEMP):
		output += prefix + "IMU Temp (degC): %.2f" % (state.imutemp) + suffix
	if (state.content & SBS_ZRANGE):
		output += prefix + "Z Range (m): %.2f" % (state.zrange) + suffix
	if (state.content & SBS_ZPRESSURE):
		output += prefix + "Z Pressure (m): %.2f" % (state.zpressure) + suffix
	if (state.content & SBS_HRANGES):
		output += prefix + "H ranges (m): "
		for i in range(0,4):
			r = sbGetFloat(state.hranges,i)
			if (r>=0xffff):
				output += "inf "
			else:
				output += "%.2f " % r
		output += suffix
	if (state.content & SBS_XY_REL):
		output += prefix + "XY Rel (m): %.2f, %.2f" % (state.xrel, state.yrel) + suffix
	if (state.content & SBS_BATTERY):
		output += prefix + "Battery (V): %.2f" % (state.battery) + suffix
	if (state.content & SBS_TIMEOUT):
		output += prefix + "Timeout (ms): WD %d CTRL %d " % (state.watchdogTimeout,state.controlTimeout) + suffix
	if (state.content & SBS_O_ATTITUDE):
		output += prefix + "Output Attitude: %04X, %04X, %04X" % (state.o_attitude[0], state.o_attitude[1], state.o_attitude[2]) + suffix
	if (state.content & SBS_O_ALTITUDE):
		output += prefix + "Output Altitude: %04X" % ( state.o_altitude) + suffix
	if (state.content & SBS_O_TOL):
		output += prefix + "Output TOL: %04X" % (state.o_tol) + suffix
	if (state.content & SBS_O_XY):
		output += prefix + "Output XY: %04X, %04X" % (sbGetShort(state.o_xy,0), sbGetShort(state.o_xy,1)) + suffix
	if (state.content & SBS_O_OAVOID):
		output += prefix + "Output OA XY: %04X, %04X" % ( sbGetShort(state.o_oavoid,0), sbGetShort(state.o_oavoid,1)) + suffix
	output += endlist
	return output

def create_status(state,html):
	output = ''
	if html:
		return create_status_gen(state,"<ul>","</ul>","<li>","</li>")
	else:
		return create_status_gen(state,"\n","","","\n")


def test_basic_function():
	res = sbSimpleInitialise(heliconn)
	print "sbSimpleInitialise: %d" % res
	res = sbSimpleWaitState(heliconn,None,1.0)
	print "sbSimpleWaitState: %d" % res
	text = create_status(heliconn.state,False)
	print text
	res = sbSimpleTerminate(heliconn)
	print "sbSimpleTerminate: %d" % res

def fork_and_closefid(command):
	pid = os.fork()
	if pid:
		return pid
	else:
		for x in range(3,256):
			try:
				os.close(x)
				print "Closing port %d" % x
			except OSError:
				pass
		cline = command.split(" ")
		os.execvp(cline[0],cline) 

class BootloaderServer(SimpleHTTPServer.SimpleHTTPRequestHandler):

	def send_cmd_status(self,success,text=""):
		response = 404
		message = "FAILURE"
		if success:
			response = 200
			message = "SUCCESS"
		if len(text) > 0:
			message += ": " + text
		self.protocol_version = "HTTP/1.2"
		self.send_response(response)
		self.send_header("Content-Type","text/plain")
		self.send_header("Content-Length",len(message))
		self.send_header("Cache-Control", "no-cache")
		self.send_header("Pragma", "no-cache")
		self.send_header("Expires", -1)
		self.end_headers()
		self.wfile.write(message)

	def send_content(self,type,message):
		self.protocol_version = "HTTP/1.2"
		self.send_response(200)
		self.send_header("Content-Type","text/"+type)
		self.send_header("Content-Length",len(message))
		self.send_header("Cache-Control", "no-cache")
		self.send_header("Pragma", "no-cache")
		self.send_header("Expires", -1)
		self.end_headers()
		self.wfile.write(message)

	def filename(self,text):
		l = text.split("/")
		if len(l) == 0:
			return ""
		else:
			return l[len(l)-1]


	def do_GET(self):
		global coax_device
		global coax_port
		global video_pid
		global repeater_pid
		global heliconn
		global processingReq
		if processingReq:
			self.send_cmd_status(False,"Busy")
			return
		processingReq = True
		try:
			if (self.path == '/command/hello'):
				message = "<html> <body> <h1>Hello to you</h1></body></html>"
				self.send_content("html",message)
			elif (self.path == '/command/disconnect'):
				if heliconn == None:
					self.send_cmd_status(True)
					raise Exception("Disconnected")
				res = sbSimpleTerminate(heliconn)
				if res == 0:
					self.send_cmd_status(True)
				else:
					self.send_cmd_status(False, "Disconnection failed")
				sbSimpleFree(heliconn)
				heliconn = None
			elif (self.path == '/command/connect'):
				if heliconn != None:
					self.send_cmd_status(True)
					raise Exception("Connected")
				heliconn = sbSimpleAlloc(coax_device,coax_port)
				heliconn.masterMode = 0
				res = sbSimpleInitialise(heliconn)
				if res == 0:
					self.send_cmd_status(True)
				else:
					self.send_cmd_status(False, "Initialisation failed")
					sbSimpleFree(heliconn)
					heliconn = None
			elif (self.path == '/command/startvideo'):
				try:
					if video_pid <= 0:
						video_pid = fork_and_closefid(video_cmd)
					self.send_cmd_status(True)
				except:
					traceback.print_exc()
					self.send_cmd_status(False, "Failed to start video transmission")
				print "Video started: %d" % video_pid
			elif (self.path == '/command/stopvideo'):
				print "Trying to kill video %d" % video_pid
				try:
					os.kill(video_pid,signal.SIGTERM)
					time.sleep(0.5)
					# os.kill(video_pid,signal.SIGKILL)
					os.waitpid(video_pid,0)
					video_pid = -1
					self.send_cmd_status(True)
				except:
					traceback.print_exc()
					self.send_cmd_status(False, "Failed to kill video")
			elif (self.path == '/command/startrepeater'):
				try:
					if repeater_pid <= 0:
						repeater_pid = fork_and_closefid(repeater_cmd)
					self.send_cmd_status(True)
				except:
					traceback.print_exc()
					self.send_cmd_status(False, "Failed to start repeater")
				print "Repeater started: %d" % repeater_pid
			elif (self.path == '/command/stoprepeater'):
				print "Trying to kill repeater %d" % repeater_pid
				try:
					os.kill(repeater_pid,signal.SIGTERM)
					time.sleep(0.5)
					# os.kill(repeater_pid,signal.SIGKILL)
					os.waitpid(repeater_pid,0)
					repeater_pid = -1
					self.send_cmd_status(True)
				except:
					traceback.print_exc()
					self.send_cmd_status(False, "Failed to kill repeater")
			elif re.match('/command/setverbose\?[0-9]',self.path):
				if heliconn == None:
					self.send_cmd_status(False, "Disconnected")
					raise Exception("Disconnected")
				value = self.path.split("?")[1];
				res = sbConfigureCommLoop(heliconn.control,int(value),0)
				if res == 0:
					self.send_cmd_status(True)
				else:
					self.send_cmd_status(False, "Set verbose failed")
			elif re.match('/command/settimeout\?[0-9]+&[0-9]+',self.path):
				if heliconn == None:
					self.send_cmd_status(False, "Disconnected")
					raise Exception("Disconnected")
				value = self.path.split("?")[1].split("&")
				ctrlto = value[0]
				wdto = value[1]
				res = sbConfigureTimeout(heliconn.control,int(wdto),int(ctrlto))
				if res == 0:
					self.send_cmd_status(True)
				else:
					self.send_cmd_status(False, "Set timeout failed")
			elif (self.path == '/command/status'):
				if heliconn == None:
					self.send_cmd_status(False, "Disconnected")
					raise Exception("Disconnected")
				res = sbSimpleWaitState(heliconn,None,1.0)
				if res == 0:
					self.send_content("plain",create_status(heliconn.state,False))
				else:
					self.send_cmd_status(False, "Did not receive state update")
			elif (self.path == '/command/status.html'):
				if heliconn == None:
					self.send_cmd_status(False, "Disconnected")
					raise Exception("Disconnected")
				res = sbSimpleWaitState(heliconn,None,1.0)
				if res == 0:
					self.send_content("html",create_status(heliconn.state,True))
				else:
					self.send_cmd_status(False, "Did not receive state update")
			else:
				print "Un-treated URL: " + self.path
				SimpleHTTPServer.SimpleHTTPRequestHandler.do_GET(self)
		except:
			traceback.print_exc()
			pass
		processingReq = False




def run(server_class=BaseHTTPServer.HTTPServer,
        handler_class=BootloaderServer):
    server_address = ('', 8081) 
    httpd = server_class(server_address, handler_class)
    httpd.serve_forever()

try:
	run()
	# test_basic_function()
except KeyboardInterrupt:
	print "Terminated"

if heliconn:
	print "Freeing heliconn"
	sbSimpleFree(heliconn)

if repeater:
	repeater.terminate()
	time.sleep(0.1)
	repeater.kill()


