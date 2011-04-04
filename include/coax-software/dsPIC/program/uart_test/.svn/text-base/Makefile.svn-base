STDLIB=/home/cedricp/src/e-puck/library/std_microchip
COMLIB=../../../../communication
COAXLIB=../../library
PREFIX=pic30-elf-
AR=$(PREFIX)ar
CC=$(PREFIX)gcc
CPU=33FJ256GP506
#CPU=30F6014A
DEFINES=-DPIC30 -DCOAX -DSBC_HAS_COMM
CCOPTS=-Wall -c -mcpu=$(CPU)  -g $(DEFINES) \
	-I$(COAXLIB)/include -I$(COAXLIB)/include/uart \
	-I$(STDLIB)/include/ \
	-I$(STDLIB)/support/h/ \
	-I$(STDLIB)/support/peripheral_30F_24H_33F/ 

AS=$(PREFIX)as
ASOPTS=-g \
	   -I$(STDLIB)/include -I$(STDLIB)/support/inc/ -p$(CPU)
BIN2HEX=$(PREFIX)bin2hex
BIN2HEXOPTS=
LD=$(PREFIX)ld
LKR=$(STDLIB)/std_microchip/support/gld/p$(CPU).gld
LDOPTS= -Map advanced_sercom.map \
	-L$(COAXLIB)/lib \
		-L$(STDLIB)/lib/ -L$(STDLIB)/lib/dsPIC33F -T$(STDLIB)/support/gld/p$(CPU).gld \
		-heap 512 
LDLIBS=-lc-elf -lm-elf -lpic30-elf  -lp33FJ256GP506-elf

TARGET_SRC=main.c 

TARGET=Uart_test_CoaX

all: $(TARGET).hex

$(TARGET).hex: $(TARGET).bin 
	$(BIN2HEX) $<

$(TARGET).bin:$(TARGET_SRC:%.c=%.o) 
	$(LD) $(LDOPTS) -o$@ $+  \
		-lled -luart -lagenda -lutils -lconfigs $(LDLIBS)

$(TARGET).a: $(TARGET_SRC:%.c=%.o)
	rm -f $@
	$(AR) rsu $@ $+



$(TARGET_SRC:%.c=%.o): %.o : %.c
	$(CC) $(CCOPTS) -o $@ $<

clean:
	-rm -f $(TARGET_SRC:%.c=%.o)
	-rm -f $(TARGET).bin
	-rm -f $(TARGET).hex
