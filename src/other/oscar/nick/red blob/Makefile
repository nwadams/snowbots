# Makefile - to compile  

CC=g++

#INCLUDE= -I/usr/local/include/opencv
CFLAGS= -Wall -Werror -g
LDFLAGS= -lm -lcv -lcxcore -lhighgui
#LIB= -L/usr/local/lib
OBJ= main3.o
EXE= main3 main2 main

default: all

all: $(EXE)

%.o : %.cpp
	$(CC) $(INCLUDE) $(CFLAGS) -c $<

main3: main3.o
	$(CC) $(LIB) $(CFLAGS) $^ -o $@ $(LDFLAGS)


clean:
	rm -f *.o $(EXE)
