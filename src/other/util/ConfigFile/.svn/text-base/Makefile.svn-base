# Makefile for ConfigFile class
# code from: http://www-personal.umich.edu/~wagnerr/ConfigFile.html

#this Makefile relies on a macro "REPOS" set by the master makefile.  It contains
#the path to the root folder of the repository, the folder that contains the "vision"
#directory

CC = g++
CFLAGS = -Wall -ansi

LIBDEST = $(REPOS)/lib/cpp
HDRDEST = $(REPOS)/include/ConfigFile

all: example tester library

example: example.cpp ConfigFile.cpp
	$(CC) $(CFLAGS) -o example example.cpp ConfigFile.cpp

tester: tester.cpp ConfigFile.cpp
	$(CC) $(CFLAGS) -o tester tester.cpp ConfigFile.cpp

ConfigFile.o: ConfigFile.cpp ConfigFile.h Triplet.h
	$(CC) $(CFLAGS) -c ConfigFile.cpp

library: ConfigFile.o
	ar rcs ConfigFile.a ConfigFile.o
	cp ConfigFile.a $(LIBDEST)/
	cp ConfigFile.h $(HDRDEST)/

run: example
	./example | tee example.out

test: tester
	./tester | tee test.out

tidy:
	@ rm -f *.o

clean: tidy
	@ rm -f example tester example.out test.out core*
