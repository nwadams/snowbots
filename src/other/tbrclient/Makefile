#Makefile for the tbrclient class

SNOWBOTS_ROOT = $(realpath ../..)
SNOWBOTS_SRC = $(SNOWBOTS_ROOT)/src
SNOWBOTS_LIB = $(SNOWBOTS_ROOT)/lib
SNOWBOTS_INC = $(SNOWBOTS_ROOT)/include
SNOWBOTS_BIN = $(SNOWBOTS_ROOT)/bin

LIBDEST = $(SNOWBOTS_LIB)
INCDEST = $(SNOWBOTS_INC)/tbrclient
BINDEST = $(SNOWBOTS_BIN)

DBG    = -g
OPT    = -O3
CPP    = g++
CFLAGS = $(DBG) $(OPT) -I$(SNOWBOTS_INC)

HDR = tbrclient.h
OBJ = tbrclient.o
LIB = tbrclient.a
APPS = tbrdiag tbrkill

all: $(APPS)

default: $(LIB)

$(LIB): $(OBJ)
	ar rcs $(LIB) $(OBJ)
	cp $(LIB) $(LIBDEST)/
	cp $(HDR) $(INCDEST)/

$(OBJ): tbrclient.cpp $(HDR)

.cpp.o:
	$(CPP) $(CFLAGS) -c $< -o $@

tbrdiag: tbrdiag.o $(LIB)
	$(CPP) tbrdiag.o -o $@ ./$(LIB)
	cp $@ $(BINDEST)/

tbrkill: tbrkill.o $(LIB)
	$(CPP) $< -o $@ ./$(LIB)
	cp $@ $(BINDEST)/

clean:
	rm -f *.o $(LIB) $(APPS)
	rm -f *~
	rm -f $(LIBDEST)/$(LIB)
	rm -f $(INCDEST)/$(HDR)
	rm -f $(BINDEST)/tbrdiag $(BINDEST)/tbrkill
