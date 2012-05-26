#Makefile for the PID throttle and PID classes

#this Makefile relies on a macro "REPOS" set by the master makefile.  It contains
#the path to the root folder of the repository, the folder that contains the "vision"
#directory

INCDIR = $(REPOS)/include
LIBDIR = $(REPOS)/lib

INCPATHS = -I$(INCDIR)/pid
LIBPATHS = -L$(LIBDIR)

INCDEST = $(INCDIR)/pid
LIBDEST = $(LIBDIR)/cpp

CPP = g++
OPT = -O3

HDR = pid.h pidthrottle.h
OBJ = pid.o pidthrottle.o
LIBS = 
LIBNAME = pid.a
BIN = throttleTest

CFLAGS = $(DBG) $(OPT) $(INCPATHS)


# ==========================================================================

.cpp.o:
	$(CPP) $(CFLAGS) -c $< -o $@

default: $(BIN)

main.o: main.cpp 

$(LIBNAME): $(OBJ) 
	ar rcs $(LIBNAME) $(OBJ)
	cp $(LIBNAME) $(LIBDEST)/
	cp pid.h $(INCDEST)/
	cp pidthrottle.h $(INCDEST)/
	
$(BIN): $(LIBNAME) main.o 
	$(CPP) $(CFLAGS) -o $(BIN) main.o $(LIBDIR)/$(LIBNAME) $(LIBPATHS) $(LIBS)

clean:
	/bin/rm -f $(BIN) $(LIBNAME) *.o
	/bin/rm -f $(LIBDEST)/$(LIBNAME)
	/bin/rm -f $(INCDEST)/pid.h
	/bin/rm -f $(INCDEST)/pidthrottle.h

clean-all: clean
	/bin/rm -f *~
