# Makefile - to compile  

CC=gcc

# Using default robot config traxxas
# Use make 'ROBOT=traxxas|duratrax|snowfury'

ifndef $(ROBOT)
	ROBOT=traxxas
endif

# INCLUDE= -I/usr/local/include/opencv
CFLAGS= -Wall -g -pthread -D$(ROBOT)
# LDFLAGS= -lm -lcv -lcxcore -lhighgui
# LIB= -L/usr/local/lib -L/usr/lib -lqt-mt  -lSM -lICE  -lX11 -lXext -lXmu -lXt -lXi -lcwiid
LIB= -L/usr/local/lib -L/usr/lib  -lSM -lICE  -lX11 -lXext -lXmu -lXt -lXi -lcwiid
# -lcwiid for wiimote library
OBJLINK=
OBJ= 
EXE=eStopLite

default: all

all: $(EXE)

%.o : %.c
	$(CC) $(INCLUDE) $(CFLAGS) -c $<

eStopLite:  eStopLite
	$(CC) $(LIB) $(CFLAGS) $^ -o $@ $(LDFLAGS)
.PHONY : clean
clean:
	rm -f *.o $(EXE)
