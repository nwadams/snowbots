# Makefile - to compile  

CC=gcc

# Using default robot config traxxas
# Use make 'ROBOT=traxxas|duratrax|snowfury'

ifndef $(ROBOT)
	ROBOT=traxxas
endif

INCLUDE= -I/usr/local/include/opencv
CFLAGS= -Wall -g -pthread -D$(ROBOT)
LDFLAGS= -lm -lcv -lcxcore -lhighgui
LIB= -L/usr/local/lib
OBJLINK=tbrprobe_client.o quartz_drag_race.o
OBJ=tbrprobe_client.o quartz_drag_race.o
EXE=  quartz quartz_drag_race quartz02 quartz03 quartz04 stopsign 

default: all

all: $(EXE)

%.o : %.c
	$(CC) $(INCLUDE) $(CFLAGS) -c $<

quartz_drag_race : quartz_drag_race.o tbrprobe_client.o
	$(CC) $(LIB) $(CFLAGS) $^ -o $@ $(LDFLAGS)
quartz04: quartz04.o
	$(CC) $(LIB) $(CFLAGS) $^ -o $@ $(LDFLAGS)
quartz03: quartz03.o
	$(CC) $(LIB) $(CFLAGS) $^ -o $@ $(LDFLAGS)
quartz02: quartz02.o
	$(CC) $(LIB) $(CFLAGS) $^ -o $@ $(LDFLAGS)
quartz: quartz.o
	$(CC) $(LIB) $(CFLAGS) $^ -o $@ $(LDFLAGS)
stopsign: stopsign.o
	$(CC) $(LIB) $(CFLAGS) $^ -o $@ $(LDFLAGS)
.PHONY : clean
clean:
	rm -f *.o $(EXE)
