# Makefile - to compile  

CC=g++

#INCLUDE= -I/usr/local/include/opencv
CFLAGS= -Wall -Werror -g
LDFLAGS= -lm -lcv -lcxcore -lhighgui
#LIB= -L/usr/local/lib
OBJ= main.o
EXE= vision

default: all

all: $(EXE)

%.o : %.cpp
	$(CC) $(INCLUDE) $(CFLAGS) -c $<

vision: main.o
	$(CC) $(LIB) $(CFLAGS) $^ -o $@ $(LDFLAGS)


clean:
	rm -f *.o $(EXE)
