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

LIB_S_SRC= coax_uart1_rx_char.s coax_uart1_tx_char.s \
	coax_uart2_rx_char.s coax_uart2_tx_char.s
#LIB_C_SRC= coax_bluetooth.c uart.c
#LIB_C_HDR= coax_bluetooth.h coax_uart.h
LIB_C_SRC= uart.c
LIB_C_HDR= coax_uart.h

LIB_C_OBJS=  $(LIB_C_SRC:%.c=%_pic30.o)
LIB_S_OBJS=  $(LIB_S_SRC:%.s=%_pic30.o)

TARGET=libuart.a

all: $(TARGET)

$(TARGET): $(LIB_C_OBJS) $(LIB_S_OBJS)
	rm -f $@
	$(AR) rsu $@ $+


$(LIB_C_OBJS): %_pic30.o : %.c
	$(CC) $(CCOPTS) -o $@ $<

$(LIB_S_OBJS): %_pic30.o : %.s
	$(CC) $(CCAOPTS) -o $@ $<


olist:
	@echo "$(LIB_C_OBJS) $(LIB_S_OBJS)"


install: $(TARGET)
	mkdir -p $(DESTDIR)/lib
	mkdir -p $(DESTDIR)/include/uart
	cp $(TARGET) $(DESTDIR)/lib
	cp $(LIB_C_HDR) $(DESTDIR)/include/uart

echo:
	@echo $(LIB_AS_SRC_ALL)

clean:
	-rm -f $(LIB_OBJS)
	-rm -f $(TARGET)
