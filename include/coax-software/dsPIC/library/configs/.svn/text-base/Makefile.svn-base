# *************************************************************
# 
#  API and Communication library for the Skybotix AG
#  helicopters, and particularly COAX
# 
#  Developed by Cedric Pradalier: cedric.pradalier@skybotix.ch
#  Send modification or corrections as patches (diff -Naur)
#  Copyright: Skybotix AG, 2009-2012
#  
# *************************************************************

LIB_C_SRC= coax_config.c init_port.c
LIB_C_HDR= coax_config.h  coax_ports.h  init_port.h

LIB_OBJS=  $(LIB_C_SRC:%.c=%_pic30.o)

TARGET=libconfigs.a

all: $(TARGET)

$(TARGET): $(LIB_OBJS)
	rm -f $@
	$(AR) rsu $@ $+


$(LIB_OBJS): %_pic30.o : %.c
	$(CC) $(CCOPTS) -o $@ $<


olist:
	@echo "$(LIB_OBJS)"


install: $(TARGET)
	mkdir -p $(DESTDIR)/lib
	mkdir -p $(DESTDIR)/include/configs
	cp $(TARGET) $(DESTDIR)/lib
	cp $(LIB_C_HDR) $(DESTDIR)/include/configs

echo:
	@echo $(LIB_AS_SRC_ALL)

clean:
	-rm -f $(LIB_OBJS)
	-rm -f $(TARGET)
