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

TARGET_SRC=control.c state.c timeouts.c globals.c bluetooth.c

ALL_SRC=control-coax003x.c control-coax002x.c $(TARGET_SRC)
COAX003X_SRC=control-coax003x.c $(TARGET_SRC)
COAX002X_SRC=control-coax002x.c $(TARGET_SRC)

TARGET=controller-coax003x-2401 controller-coax002x-2401 \
	   controller-coax003x-2402 controller-coax002x-2402 reset_mc
		

all: $(TARGET:%=%.hex)

%.hex : %.bin 
	$(BIN2HEX) $<

controller-coax003x-2401.bin:$(COAX003X_SRC:%.c=%.o) main-coax003x-2401.o
	$(LD) $(LDOPTS) -o$@ $+  -lsbcom_pic30 \
		-lcoax $(LDLIBS)

controller-coax003x-2402.bin:$(COAX003X_SRC:%.c=%.o) main-coax003x-2402.o
	$(LD) $(LDOPTS) -o$@ $+  -lsbcom_pic30 \
		-lcoax $(LDLIBS)

controller-coax002x-2401.bin:$(COAX002X_SRC:%.c=%.o) main-coax002x-2401.o
	$(LD) $(LDOPTS) -o$@ $+  -lsbcom_pic30 \
		-lcoax $(LDLIBS)

controller-coax002x-2402.bin:$(COAX002X_SRC:%.c=%.o) main-coax002x-2402.o
	$(LD) $(LDOPTS) -o$@ $+  -lsbcom_pic30 \
		-lcoax $(LDLIBS)

reset_mc.bin:reset_mc.o
	$(LD) $(LDOPTS) -o$@ $+ -lcoax $(LDLIBS)

$(ALL_SRC:%.c=%.o): %.o : %.c
	$(CC) $(CCOPTS) -o $@ $<

reset_mc.o: reset_mc.c
	$(CC) $(CCOPTS) -o $@ $<

main-coax003x-2401.o: main_com.c
	$(CC) -DCOAX003X -DRC2401 $(CCOPTS) -o $@ $<

main-coax003x-2402.o: main_com.c
	$(CC) -DCOAX003X -DRC2402 $(CCOPTS) -o $@ $<

main-coax002x-2401.o: main_com.c
	$(CC) -DCOAX002X -DRC2401 $(CCOPTS) -o $@ $<

main-coax002x-2402.o: main_com.c
	$(CC) -DCOAX002X -DRC2402 $(CCOPTS) -o $@ $<

clean:
	-rm -f *.o *.a
	-rm -f *.bin
	-rm -f *.hex
