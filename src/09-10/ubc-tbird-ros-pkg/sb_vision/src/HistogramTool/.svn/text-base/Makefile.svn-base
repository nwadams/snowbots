# Makefile - to compile  

CC=g++

INCLUDE= -I/usr/local/include/opencv
CFLAGS= -Wall -Werror -g
LDFLAGS= -lm -lcv -lcxcore -lhighgui
LIB= -L/usr/local/lib
OBJ= driver.o flavours.o
EXE= vision

default: all

all: $(EXE)

%.o : %.c
	$(CC) $(INCLUDE) $(CFLAGS) -c $<

vision: driver.o flavours.o
	$(CC) $(LIB) $(CFLAGS) $^ -o $@ $(LDFLAGS)


clean:
	rm -f *.o $(EXE)
