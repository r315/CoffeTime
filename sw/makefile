#
#	MAKEFILE FOR HI-TECH COMPILER
#	MCU = PIC16F73
#

SRC = coffetime.c pid.c boiler.c
PRJ = coffetime
MCU = 16F76
CC = picc.exe

CFLAGS =-q -g -P --char=unsigned --asmlist --chip=$(MCU)
CFLAGS +="--errformat=%f:%l: Error: %s"  
CFLAGS +="--warnformat=%f:%l: Warning: %s"
CFLAGS += -I"..\..\include"

all:
	$(CC) $(CFLAGS) -C $(SRC)
	$(CC) $(CFLAGS) -o$(PRJ).cof $(SRC:.c=.obj)

clean:
	rm -rf *.obj *.rlf *.sdb *.lst *.cof *.hxl *.map *.sym *.hex
	


