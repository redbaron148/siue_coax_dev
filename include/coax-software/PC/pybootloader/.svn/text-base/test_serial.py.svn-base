#!/usr/bin/python

import serial
import time
import sys
serial = serial.Serial("/dev/ttyUSB0",115200,timeout=0.01)

while True:
	serial.write(chr(0xAA))
	sys.stdout.write('.')
	sys.stdout.flush()
	while True:
		c = serial.read(1)
		if len(c) == 0:
			break
		sys.stdout.write(c)
	sys.stdout.flush()
	time.sleep(0.001)


