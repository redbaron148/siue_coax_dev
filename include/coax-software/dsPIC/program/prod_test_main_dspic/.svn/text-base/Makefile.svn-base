STDLIB=/usr/pic30-elf
SUPPORTLIB=/usr/share/pic30-support
COMLIB=../../../..
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
# 	Executing: "C:\Program Files\Microchip\MPLAB C30\bin\pic30-gcc.exe" 
# 		-omf=elf -mcpu=33FJ256GP506 -x c -c "..\..\library\analog\analog.c" -o".\obj\analog.o" -I"..\..\library" -g -Wall -mlarge-data
# 	Executing: "C:\Program Files\Microchip\MPLAB C30\bin\pic30-gcc.exe" \
# 		-omf=elf -mcpu=33FJ256GP506 "obj\main.o" "obj\coax_led.o" "obj\coax_config.o" "obj\uart.o" "obj\utils.o" "obj\init_port.o" "obj\coax_uart2_tx_char.o" "obj\coax_uart1_rx_char.o"
# 		"obj\coax_uart1_tx_char.o" "obj\coax_uart2_rx_char.o" "obj\analog.o" -o"Uart_test_CoaX.elf" 
# 		-Wl,-L"..\..\library",-L"C:\Program Files\Microchip\MPLAB C30\lib",-Tp33FJ256GP506.gld,--defsym=__MPLAB_BUILD=1,--heap=512,-Map="Uart_test_CoaX.map",--report-mem,--cref
CCOPTS=-mcpu=$(CPU) -x c  -g -Wall $(DEFINES)\
	-mresource=$(SUPPORTLIB)/c30_device.info -mlarge-data\
	-I$(SUPPORTLIB)/support/generic/h \
	-I$(SUPPORTLIB)/support/peripheral_30F_24H_33F \
	-I$(STDLIB)/include/ \
 	-I$(COMLIB)/include -I$(COAXLIB)/include

TARGET=controller

AS=$(PREFIX)as
ASOPTS=-g \
	   -I$(STDLIB)/include -I$(STDLIB)/support/inc/ -p$(CPU)
BIN2HEX=$(PREFIX)bin2hex
BIN2HEXOPTS=
LD=$(PREFIX)ld
LKR=$(STDLIB)/support/gld/p$(CPU).gld
LDOPTS= -Map $(TARGET).map \
	-L$(COMLIB)/lib -L$(COAXLIB)/lib \
		-L$(STDLIB)/lib/ -L$(STDLIB)/lib/dsPIC33F \
		-T$(SUPPORTLIB)/support/gld/p$(CPU).gld \
		-heap 512 
LDLIBS=-lc-elf -lm-elf -lpic30-elf  -lp33FJ256GP506-elf

TARGET_SRC=main.c e_bluetooth.c


all: $(TARGET).hex

$(TARGET).hex: $(TARGET).bin 
	$(BIN2HEX) $<

$(TARGET).bin:$(TARGET_SRC:%.c=%.o) 
	#$(LD) $(LDOPTS) -o$@ $+ -lcoax $(LDLIBS)
	$(CC) -o $@ $+ -Wl,-L$(COMLIB)/lib,-L$(COAXLIB)/lib,-L$(STDLIB)/lib/,-L$(STDLIB)/lib/dsPIC33F,-lcoax,-T$(SUPPORTLIB)/support/gld/p$(CPU).gld,--defsym=__MPLAB_BUILD=1,--heap=512,-Map="Uart_test_CoaX.map",--report-mem,--cref


$(TARGET).a: $(TARGET_SRC:%.c=%.o)
	rm -f $@
	$(AR) rsu $@ $+



$(TARGET_SRC:%.c=%.o): %.o : %.c
	$(CC) $(CCOPTS) -S -o $(@:%.o=%.s) $<
	$(CC) $(CCOPTS) -c -o $@ $<

clean:
	-rm -f $(TARGET_SRC:%.c=%.o)
	-rm -f $(TARGET).bin
	-rm -f $(TARGET).hex
