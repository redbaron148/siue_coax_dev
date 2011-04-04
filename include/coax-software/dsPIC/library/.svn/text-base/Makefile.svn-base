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
DIRS= agenda i2c led motor rc imu control \
	spi us sharp utils analog configs uart mouse

export ELIB=/usr/pic30-elf
export PREFIX=pic30-elf-
#export AR=$(PREFIX)ar
export RANLIB=$(PREFIX)ranlib
#export RANLIB=true
export AR=ar
export CC=$(PREFIX)$(PREFIX)gcc
export AS=$(PREFIX)as
export STDLIB=/usr/pic30-elf
export SUPPORTLIB=/usr/share/pic30-support

# Optimisation option here should be set to -O0, because variable used in
# interrupt function are not yet all properly declared volatile, so -O1 or -O2
# would remove them from the loop where they are polled.
export CCOPTS=-c -x c -mcpu=$(CPU) -O0 -mlarge-data -g -Wall \
	-mresource=$(SUPPORTLIB)/c30_device.info \
	-I$(SUPPORTLIB)/support/generic/h \
	-I$(SUPPORTLIB)/support/peripheral_30F_24H_33F \
	-I$(ELIB)/include/ -I.. 

export ASOPTS=-p=$(CPU)  -g \
	-I$(SUPPORTLIB)/support/inc 

export CCAOPTS=-c -Wa,-p=$(CPU),-g,-I$(SUPPORTLIB)/support/inc 


export DESTDIR=..

# export CPU=30F6014A
export CPU=33FJ256GP506

LIBALL=libcoax.a

all: $(DIRS:%=%.all) $(LIBALL)

$(LIBALL): $(DIRS:%=%.all) libclean $(DIRS:%=%.liball)
	$(RANLIB) $@

.PHONY:libclean
libclean: 
	rm -f $(LIBALL)

clean: $(DIRS:%=%.clean)
	rm -f $(LIBALL)

install: $(DIRS:%=%.install)
	cp -f $(LIBALL) lib
	$(RANLIB) lib/$(LIBALL)

.PHONY:$(DIRS:%=%.all)
$(DIRS:%=%.all): %.all : %
	$(MAKE) -C $<

.PHONY:$(DIRS:%=%.clean)
$(DIRS:%=%.clean): %.clean : %
	$(MAKE) -C $< clean

.PHONY:$(DIRS:%=%.install)
$(DIRS:%=%.install): %.install : %
	$(MAKE) -C $< install

.PHONY:$(DIRS:%=%.liball)
$(DIRS:%=%.liball): %.liball : %
	@echo "In $<"
	$(AR) rsu $(LIBALL) $(foreach file,$(shell $(MAKE) -s -C $< olist 2>/dev/null),$</$(file))



