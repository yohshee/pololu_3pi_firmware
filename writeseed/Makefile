DEVICE = atmega328p
MCU = atmega328p
AVRDUDE_DEVICE = m328p
DEVICE ?= atmega168
MCU ?= atmega168
AVRDUDE_DEVICE ?= m168

CFLAGS=-g -Wall -mcall-prologues -mmcu=$(MCU) $(DEVICE_SPECIFIC_CFLAGS) -Os
CC=avr-gcc
OBJ2HEX=avr-objcopy 
LDFLAGS=-Wl,-gc-sections -lm -lpololu_$(DEVICE) -Wl,-relax

PORT ?= /dev/ttyACM0
AVRDUDE=avrdude

LFUSE=0xf6
HFUSE=0xd1
EFUSE=0x04

TARGET=driver
OBJECT_FILES=driver.o

all: $(TARGET).hex

clean:
	rm -f *.o *.hex *.obj *.hex

%.hex: %.obj
	$(OBJ2HEX) -R .eeprom -O ihex $< $@

driver.o: driver.c

%.obj: $(OBJECT_FILES)
	$(CC) $(CFLAGS) $(OBJECT_FILES) $(LDFLAGS) -o $@

readfuses: 
	$(AVRDUDE) -p $(AVRDUDE_DEVICE) -c avrisp2 -P $(PORT) -U hfuse:r:high.txt:h -U lfuse:r:low.txt:h -U efuse:r:ext.txt:h

writefuses:
	$(AVRDUDE) -p $(AVRDUDE_DEVICE) -c avrisp2 -P $(PORT) -U lfuse:w:$(LFUSE):m -U hfuse:w:$(HFUSE):m -U efuse:w:$(EFUSE):m

program: $(TARGET).hex
	$(AVRDUDE) -p $(AVRDUDE_DEVICE) -c avrisp2 -P $(PORT) -U flash:w:$(TARGET).hex
