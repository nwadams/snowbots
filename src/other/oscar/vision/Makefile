# Makefile - to compile  

CC=g++

#INCLUDE= -I/usr/local/include/opencv
CFLAGS= -Wall -Werror -g
LDFLAGS= -lm -lcv -lcxcore -lhighgui
#LIB= -L/usr/local/lib
OBJ=path.o video-path.o
EXE=path video-path

default: all

all: $(EXE)

%.o : %.cpp
	$(CC) $(INCLUDE) $(CFLAGS) -c $<


path: path.o
	$(CC) $(LIB) $(CFLAGS) $^ -o $@ $(LDFLAGS)



clean:
	rm -f *.o $(EXE)
