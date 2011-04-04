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

class ParsedLine:
    length = 0
    address = 0
    type = 0
    checksum = 0
    content = ''

    def __init__(self,line):
        self.length = int(line[1:3],16)
        self.address = int(line[3:7],16)
        self.type = int(line[7:9],16)
        if (self.type == 0):
            for i in range(0,self.length):
                self.content += (chr(int(line[(9+2*i):(11+2*i)],16)))
        elif (self.type == 4):
            self.content = int(line[9:13],16)
        self.checksum = int(line[(9+2*self.length):(11+2*self.length)],16)

    def __repr__(self):
        return "%d bytes at %04X: type %d content %s [%02X]\n" % \
            (self.length,self.address,self.type,repr(self.content),self.checksum)


class MemoryChunk:
    length = 0
    content = chr(0) * 0x10000

    def __repr__(self):
        return ("%d bytes: " % self.length) + repr(self.content[0:200])

    def insert(self,pl,offset):
        # print "Inserting " + repr(pl)
        start = offset + pl.address
        end = start + pl.length
        if end > len(self.content):
            self.content += chr(0) * (end - len(self.content))
        self.content = self.content[0:start] + pl.content + self.content[end:]
        if (end > self.length):
            self.length = end

class UploadPacket:
    wordSize = 3
    packetWordLength = 64
    headerLength = 4
    crcLength = 1
    packetLength = headerLength + (packetWordLength * wordSize) + crcLength
    content = ''
    address = 0

    def __init__(self,address, erase=False):
        # print "Initialising packet at %06X (%s)" % (address,erase)
        self.address = address
        self.content = ''
        self.content += chr(address & 0xFF); address = address >> 8
        self.content += chr(address & 0xFF); address = address >> 8
        self.content += chr(address & 0xFF); address = address >> 8
        if erase:
            self.content += chr(0xFE)
        else:
            self.content += chr(0x60)
        self.content += chr(0) * (self.packetWordLength*self.wordSize + self.crcLength)

    def setCheckSum(self):
        sum = 0
        for i in range(0,self.packetLength-1):
            sum += ord(self.content[i])
        sum = sum & 0xFF
        if sum == 0:
            sum = chr(0)
        else:
            sum = chr(0x100 - sum)
        self.content = self.content[0:self.packetLength-1] + sum

    def setContent(self,data):
        self.content = self.content[0:4];
        for i in range(0,self.packetWordLength):
            if (4*i < len(data)-3) :
                self.content += data[(4*i):(4*i+3)]
            else:
                self.content += chr(0)*3
        # print "Packet at %06X: %d data %s" % (self.address,len(self.content),repr(data[0:0xC0]))

    def isErase(self):
        return self.content[3] == chr(0xFE)

    def __repr__(self):
        D = {True:'X', False:' '}
        out = ("%06X %s " % (self.address,D[self.isErase()]))
        for i in range(4,self.packetLength-1):
            out += "%02X " % ord(self.content[i])
        out += " | %02X" % ord(self.content[self.packetLength-1])
        return out

def packetCompare(pkt):
    if pkt.isErase():
        return pkt.address
    else:
        return pkt.address + 0.5

class HexFile:
    activeChunk = 0
    memory = MemoryChunk()
    packetList = []

    def __init__(self,fname):
        fid = open(fname)
        lines = fid.readlines()
        n = 0
        for l in lines:
            n += 1
            pl = ParsedLine(l)
            if (n % 10) == 0:
                sys.stdout.write("%")
                sys.stdout.flush()
            # print "Parsing line %d" % n
            if (pl.type == 0) :
                self.memory.insert(pl,self.activeChunk)
                # print "Added line at %06X len %d total %d\n" % (pl.address+self.activeChunk, pl.length, self.memory.length)
            elif (pl.type == 4):
                addr = pl.content * 0x10000
                if (addr > 0x40000):
                    continue
                self.activeChunk = addr
        # end for
        print ""
        fid.close()
    
    def setBootLoaderAddress(self,addr):
        newcontent=''
        newcontent += chr(addr & 0xFF); addr = addr >> 8;
        newcontent += chr(addr & 0xFF); addr = addr >> 8;
        newcontent += chr(0x04)
        newcontent += chr(0x00)
        newcontent += chr(addr & 0xFF)
        newcontent += '\x00\x00\x00'
        self.memory.content = newcontent + self.memory.content[8:]

    def createPackets(self,bootloaderStart,bootloaderEnd):
        uploadSize = 0
        self.packetList = []
        addr = 0
        while addr < self.memory.length:
            address = (addr)/2
            # print "Addr %06X %06X (%d)" % (address, addr,addr)
            packet = UploadPacket(address,False)
            packet.setContent(self.memory.content[addr:])
            packet.setCheckSum()
            if (address < bootloaderEnd) or (address > bootloaderEnd):
                self.packetList.append(packet)
                uploadSize += packet.packetWordLength
                sys.stdout.write("$")
                sys.stdout.flush()
            addr += packet.packetWordLength*(packet.wordSize+1)
        # end while
        print ""
        return uploadSize

    def createErasePackets(self):
        s = set([])
        for p in self.packetList:
            s |= set([p.address & 0xFFFC00])
        s = list(s)
        s.sort()

        for addr in s:
            packet = UploadPacket(addr,True)
            packet.setCheckSum()
            self.packetList.append(packet)
        
        self.packetList.sort(key=packetCompare)

    def reverseList(self):
        self.packetList.reverse()

    
    def getProgramAddress(self):
        code = self.memory.content[0:5]
        opcode = ord(code[2])
        if (opcode != 0x04):
            return 0x000000
        address = ord(code[4])
        address = (address << 8) | ord(code[1])
        address = (address << 8) | ord(code[0])
        return address

    
FLASH_END=0x02AC00
BOOTLOADER_START=0x02A600
PROGRAM_START=0x00200

def testModule(fname):
    h = HexFile(fname)
    print "Program at address %06X" % h.getProgramAddress()
    h.setBootLoaderAddress(BOOTLOADER_START)
    us = h.createPackets(BOOTLOADER_START,FLASH_END-1)
    packetCount = len(h.packetList)
    print "Created %d packets, %d words to upload (%d bytes)" % (packetCount, us, us*3)
    h.createErasePackets()
    print "Added %d erase packets" % (len(h.packetList) - packetCount)

    for p in h.packetList:
        print repr(p)

def processFile(fname):
    h = HexFile(fname)
    print "Program at address %06X" % h.getProgramAddress()
    h.setBootLoaderAddress(BOOTLOADER_START)
    us = h.createPackets(BOOTLOADER_START,FLASH_END-1)
    packetCount = len(h.packetList)
    print "Created %d packets, %d words to upload (%d bytes)" % (packetCount, us, us*3)
    h.createErasePackets()
    print "Added %d erase packets" % (len(h.packetList) - packetCount)
    return h.packetList


