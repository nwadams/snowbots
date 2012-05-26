# Makefile - to compile  

CC=g++
SNOWBOTS_ROOT=$(HOME)/snowbots
CFLAGS= -Wall -Werror -g
EXE=coneHugger2
LIBDIR=$(SNOWBOTS_ROOT)/lib
INCDIR=$(SNOWBOTS_ROOT)/include
TBRCLIENT=$(LIBDIR)/tbrclient.a

default: all

all: $(EXE)

%.o : %.cpp
	$(CC) $(CFLAGS) -I$(INCDIR) -c $<

$(EXE): $(EXE).o
	$(CC) $(CFLAGS) $< $(TBRCLIENT) -o $@

clean:
	rm -f *.o $(EXE)
