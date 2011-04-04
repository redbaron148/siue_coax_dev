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


import sys
import time
import serial
from HexFile import *

class ProgressReport:
    def report_stdout(self,s):
        sys.stdout.write(s)
        sys.stdout.flush()
    
    def report_string(self,s):
        print s

    def report_percent(self,percent):
        sys.stdout.write('%3.1f%%' % percent)
        sys.stdout.flush()



class SerialLoader:
    portname = ''
    filename = ''
    packets = []
    testonly = False
    def __init__(self, filename='', portname='',rtsctscfg=1):
        self.portname = portname
        self.filename = filename
        self.rtscts = rtsctscfg
        if len(self.filename)>0:
            self.packets = processFile(self.filename)
        if len(self.portname)>0:
            self.serial = serial.Serial(portname,115200,\
                    timeout=1,rtscts=self.rtscts)

    def loadFile(self,filename):
        self.filename = filename
        if len(self.filename)>0:
            self.packets = processFile(self.filename)

    def preparePort(self,portname, rtsctscfg):
        self.portname = portname
        self.rtscts = rtsctscfg
        if len(self.portname)>0:
            self.serial = serial.Serial(portname,115200,\
                    timeout=1,rtscts=self.rtscts)

    def releasePort(self):
        if self.serial:
            self.serial.close()
        self.serial = None

    def read_serial(self,num,report=ProgressReport()):
        command = ""
        while len(command)<num:
            c = self.serial.read(1)
            if len(c) == 0:
                break
            if ord(c[0]) & 0x80:
                c = chr(ord(c[0]) & 0x7f)
                if (c == 'q') or (c == 'K'):
                    command += c
                # report.report_stdout("%02X " % ord(c[0]))
            else:
                report.report_stdout(c)
        return command

    def upload(self,reqreset,report=ProgressReport(),stopclass=None):
        if reqreset:
            # Ask the board to reset it self
            self.serial.write(chr(0xC1))
            time.sleep(0.01)
            self.serial.flushInput()
        else:
            report.report_string("Reset the robot to start uploading")

        found = False
        if not self.testonly:
            for i in range(0,100):
                if stopclass and stopclass.abort():
                    report.report_string("Upload aborded")
                    return -1
                report.report_stdout('!')
                if self.rtscts:
                    self.serial.setRTS(True)
                self.serial.write(chr(0xC1))
                s = self.read_serial(2,report)
                if len(s) == 0:
                    report.report_stdout('_')
                    continue
                if (s == 'qK'):
                    found = True
                    break
                else:
                    # report.report_stdout("Read '"+s+"'\n")
                    pass
                time.sleep(0.3)
            # end for
            if not found:
                report.report_string("Bootloader did not answer, aborting")
                return -1
            
        report.report_stdout('\n')
        report.report_string("Uploading")
        nump = 0
        for p in self.packets:
            if stopclass and stopclass.abort():
                report.report_string("Upload aborded")
                return -1
            if self.testonly:
                # Desactivate real writing for GUI debug
                time.sleep(0.1)
                x = 'K'
                print "Simulating"
            else:
                # self.serial.flushInput()
                i=0
                while (i < p.packetLength):
                    end = i + 16
                    if (end > p.packetLength):
                        end = p.packetLength
                    self.serial.write(p.content[i:end])
                    # print "Written %d:%d" % (i,end)
                    i = end
                # x = self.serial.read()
                x = self.read_serial(1,report)
                # print "Read " + x
            if x == 'K':
                nump += 1
                if p.isErase():
                    report.report_stdout('X')
                else:
                    report.report_stdout('#')
                report.report_percent(float(100*nump)/len(self.packets))
            else:
                report.report_string( "Invalid answer %s" % repr(x))
                return -1
        # end for
        byebye = '\x95\x00\x00\xff'
        if not self.testonly:
            self.serial.write(byebye)
        report.report_stdout('\n')
        report.report_string("Successfull upload")
        return 0

def help(pname):
    print "\n\tUsage: %s <filename> <port> <rtscts:0/1>\n" % pname
    
        


if __name__ == "__main__":
    if len(sys.argv) == 4:
        filename = sys.argv[1]
        portname = sys.argv[2]
        rtscts = sys.argv[3]
        #testModule(filename)
        uploader = SerialLoader(filename,portname,rtscts)
        #for p in uploader.packets:
        #    print repr(p)
        uploader.upload()
    else:
        help(sys.argv[0])

