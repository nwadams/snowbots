# Makefile - to compile  

CC=gcc

INCLUDE= -I/usr/local/include/opencv
#CFLAGS= -Wall -g -pthread
LDFLAGS= -lm -lcv -lcxcore -lhighgui
#LIB= -L/usr/local/lib
OBJ=path.o
EXE=path

default: all

all: $(EXE)

%.o : %.c
	$(CC) $(INCLUDE) $(CFLAGS) -c $<

path: path.o
	$(CC) $(LIB) $(CFLAGS) $^ -o $@ $(LDFLAGS)

clean:
	rm -f *.o $(EXE)
