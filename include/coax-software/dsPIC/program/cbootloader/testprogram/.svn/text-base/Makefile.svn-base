STDLIB=/home/cedricp/src/e-puck/library/std_microchip
COMLIB=../../../../../communication
COAXLIB=../../../library
PREFIX=pic30-elf-
AR=$(PREFIX)ar
CC=$(PREFIX)gcc
CPU=33FJ256GP506
#CPU=30F6014A
DEFINES=-DPIC30 -DCOAX -DSBC_HAS_COMM
CCOPTS=-Wall -c -mcpu=$(CPU) -Os -g $(DEFINES) \
	-I$(COAXLIB)/include -I$(COAXLIB)/include/uart \
	-I$(STDLIB)/include/ \
	-I$(STDLIB)/support/h/ \
	-I$(STDLIB)/support/peripheral_30F_24H_33F/ 

AS=$(PREFIX)as
ASOPTS=-g \
	   -I$(STDLIB)/include -I$(STDLIB)/support/inc/ -p$(CPU)
STRIP=$(PREFIX)strip
BIN2HEX=$(PREFIX)bin2hex
BIN2HEXOPTS=
LD=$(PREFIX)ld
#LKR=$(STDLIB)/std_microchip/support/gld/p$(CPU).gld
LKR=upload.gld
LDOPTS= -Map main.map \
	-L$(COAXLIB)/lib \
		-L$(STDLIB)/lib/ -L$(STDLIB)/lib/dsPIC33F -T$(LKR) \
		-heap 512 
LDLIBS=-lc-elf -lm-elf -lpic30-elf  -lp33FJ256GP506-elf

TARGET_SRC=main.c 
TARGET_ASM=

TARGET=Uart_test_CoaX

all: $(TARGET).hex

$(TARGET).hex: $(TARGET).bin 
	$(STRIP) $<
	$(BIN2HEX) $<

$(TARGET).bin:$(TARGET_SRC:%.c=%.o) $(TARGET_ASM:%.s=%.o)
	$(LD) $(LDOPTS) -o $@ $+  \
		-lcoax $(LDLIBS)

$(TARGET).a: $(TARGET_SRC:%.c=%.o)
	rm -f $@
	$(AR) rsu $@ $+



$(TARGET_SRC:%.c=%.o): %.o : %.c
	$(CC) $(CCOPTS) -o $@ $<

$(TARGET_ASM:%.s=%.o): %.o : %.s
	$(AS) $(ASOPTS) -o $@ $<

clean:
	-rm -f $(TARGET_SRC:%.c=%.o)
	-rm -f $(TARGET).bin
	-rm -f $(TARGET).hex
