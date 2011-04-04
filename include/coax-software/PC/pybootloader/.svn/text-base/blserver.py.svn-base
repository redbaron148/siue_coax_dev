#!/usr/bin/python
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



import traceback
import os
import re
import BaseHTTPServer
import threading
import SimpleHTTPServer

from SerialLoader import *

bootloader = SerialLoader()
bootloader.portname="/dev/ttyS0"
bootloader.uploading = False
logqueue = ['Log started']

class LogReport(ProgressReport):
	def report_percent(self,percent):
		pass

	def report_string(self,s):
		logqueue.append(s)

class UploadThread(threading.Thread):
	def __init__(self,filename):
		threading.Thread.__init__(self)
		self.filename = filename
		self.portname = bootloader.portname

	def run(self):
		if len(bootloader.packets) == 0:
			logqueue.append('File not loaded')
			bootloader.uploading = False
			return

		# Prepare the port
		try:
			print "Trying to open " + self.portname
			logqueue.append("Trying to open " + self.portname)
			bootloader.preparePort(self.portname)
			logqueue.append('Port OK')
		except:
			traceback.print_exc()
			logqueue.append('Failed to prepare port')
			bootloader.uploading = False
			return

		logqueue.append('Reset the robot to start')
		try:
			report = LogReport()
			if bootloader.upload(report)==0:
				logqueue.append('Upload successful')
			else:
				logqueue.append('Upload failed')
		except:
			traceback.print_exc()
			logqueue.append('Upload failed')
		bootloader.releasePort()
		bootloader.uploading = False
# End class UploadThread


class BootloaderServer(SimpleHTTPServer.SimpleHTTPRequestHandler):
	def send_content(self,type,message):
		self.protocol_version = "HTTP/1.2"
		self.send_response(200)
		self.send_header("Content-Type","text/"+type)
		self.send_header("Content-Length",len(message))
		self.end_headers()
		self.wfile.write(message)

	def filename(self,text):
		l = text.split("/");
		if len(l) == 0:
			return ""
		else:
			return l[len(l)-1]

	def do_POST(self):
		print "POST"
		# print ("Headers: %s" %self.headers)
		length = self.headers['Content-Length']
		type = self.headers['Content-Type']
		match = re.match("multipart/form-data; boundary=(.*)",type)
		if match:
			boundary = match.group(1).strip()
		else:
			self.send_error(404,"Invalid content-type in input data")
			return
		# print "Boundary: " + boundary
		
		input = self.rfile.read(int(length))
		# fid = open("post.txt","w")
		# fid.write(input)
		# fid.close()

		blocks = input.split(boundary)
		# print "Found %d blocks" % len(blocks)
		bdict = {}
		def stripstr(s):
			return s.strip()

		for b in blocks:
			lines = map(stripstr,b.strip().split("\n"))
			# print "Block " + lines[0]
			match = re.match('Content-Disposition:.* name="([^"]*)".*',lines[0])
			if match:
				firstline = lines.index("") + 1
				lines = lines[firstline:]
				try:
					lastline = lines.index("")
					lines = lines[0:lastline]
				except:
					pass
				bdict[match.group(1)] = lines
				print "Block %s : %s" % (match.group(1),lines[0])
		# end for
		print "Blocks: " + repr(bdict.keys())
		try:
			filename = 'bootfiles/' + bdict["rename"][0].strip()
			fid = open(filename,"w")
			fid.write("\n".join(bdict["uploadfile"]))
			fid.close()
			print "Saved bootfile " + filename
			logqueue.append("Saved bootfile " + bdict["rename"][0].strip())
		except:
			traceback.print_exc()
			self.send_error(404,"Failed to save uploaded file")
			return
		self.send_content("html","<html><body>Upload sucessful</body></html>")

	def create_blstatus(self,html):
		output = ''
		if html:
			output='<ul>'
			output += "<li>Filename: %s</li>" % self.filename(bootloader.filename)
			output += "<li>Portname: %s</li>" % bootloader.portname
			output += "<li>Transfer size: %d packets</li>" % len(bootloader.packets)
			output += "</ul>"
		else:
			output=''
			output += "Filename: %s\n" % self.filename(bootloader.filename)
			output += "Portname: %s\n" % bootloader.portname
			output += "Transfer size: %d packets\n" % len(bootloader.packets)
		return output

	def do_GET(self):
		if (self.path == '/command/hello'):
			message = "<html> <body> <h1>Hello to you</h1></body></html>"
			self.send_content("html",message)
		elif (self.path == '/command/bootfiles'):
			output=''
			l = os.listdir('bootfiles')
			for f in l:
				if re.match('.*\.hex',f):
					output += f + "\n"
			# end for
			self.send_content("plain",output)
		elif (self.path == '/command/bootfiles.html'):
			output='<ul>'
			l = os.listdir('bootfiles')
			for f in l:
				if re.match('.*\.hex',f):
					output += "<li>" + f + "</li>"
			# end for
			output += "</ul>"
			self.send_content("html",output)
		elif (self.path == '/command/blstatus'):
			self.send_content("plain",self.create_blstatus(False))
		elif (self.path == '/command/blstatus.html'):
			self.send_content("html",self.create_blstatus(True))
		elif re.match('/command/sendfile?.+\.hex',self.path):
			self.send_content("html",self.create_blstatus(True))
			if bootloader.uploading:
				return
			fname = self.path.split("?")[1]
			bootloader.uploading = True
			self.upthread = UploadThread(fname)
			self.upthread.start()
		elif re.match('/command/selectfile?.+\.hex',self.path):
			fname = self.path.split("?")[1]
			try:
				bootloader.loadFile('bootfiles/'+fname)
				self.send_content("html",self.create_blstatus(True))
			except:
				traceback.print_exc()
				self.send_error(404,"Bootloader failed to load the file")
		elif (self.path == '/command/log'):
			l = logqueue[0:100]; l.reverse()
			output = "\n".join(l)
			self.send_content("plain",output)
		else:
			SimpleHTTPServer.SimpleHTTPRequestHandler.do_GET(self)




def run(server_class=BaseHTTPServer.HTTPServer,
        handler_class=BootloaderServer):
    server_address = ('', 8000)
    httpd = server_class(server_address, handler_class)
    httpd.serve_forever()

try:
	run()
except KeyboardInterrupt:
	print "Terminated"


