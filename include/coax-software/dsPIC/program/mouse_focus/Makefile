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
STDLIB=/usr/pic30-elf
SUPPORTLIB=/usr/share/pic30-support
COMLIB=../../../deploy/dsPIC
COAXLIB=../../library
PREFIX=pic30-elf-
AR=$(PREFIX)ar
CC=$(PREFIX)$(PREFIX)gcc
CPU=33FJ256GP506
#CPU=30F6014A
DEFINES=-DPIC30 -DCOAX -DSBC_HAS_COMM -DUSE_HARDWARE
# CCOPTS=-Wall -c -mcpu=$(CPU)  -g $(DEFINES) \
# 	-I$(COMLIB)/include -I$(COAXLIB)/include \
# 	-I$(STDLIB)/include/ \
# 	-I$(STDLIB)/support/generic/h/ -I$(STDLIB)/support/h/
CCOPTS=-c -mcpu=$(CPU)  -g -Wall $(DEFINES) -O1 \
	-mresource=/usr/share/pic30-support/c30_device.info \
	-I/usr/share/pic30-support/support/generic/h \
	-I/usr/pic30-elf/include/ \
 	-I$(COMLIB)/include -I$(COAXLIB)/include


AS=$(PREFIX)as
ASOPTS=-g \
	   -I$(STDLIB)/include -I$(STDLIB)/support/inc/ -p$(CPU)
BIN2HEX=$(PREFIX)bin2hex
BIN2HEXOPTS=
LD=$(PREFIX)ld
LKR=$(STDLIB)/support/gld/p$(CPU).gld
LDOPTS= -Map advanced_sercom.map \
	-L$(COMLIB)/lib -L$(COAXLIB)/lib \
		-L$(STDLIB)/lib/ -L$(STDLIB)/lib/dsPIC33F \
		-T$(SUPPORTLIB)/support/gld/p$(CPU).gld \
		-heap 512 
LDLIBS=-lc-elf -lm-elf -lpic30-elf  -lp33FJ256GP506-elf

TARGET_SRC=main_com.c

TARGET=mouse_focus
		

all: $(TARGET:%=%.hex)

%.hex : %.bin 
	$(BIN2HEX) $<

$(TARGET).bin:$(TARGET_SRC:%.c=%.o) 
	$(LD) $(LDOPTS) -o$@ $+  -lcoax $(LDLIBS)

$(TARGET_SRC:%.c=%.o): %.o : %.c
	$(CC) $(CCOPTS) -o $@ $<


clean:
	-rm -f *.o *.a
	-rm -f *.bin
	-rm -f *.hex
