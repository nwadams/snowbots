# Makefile for the Orange Cone line detector
# Author: Matthew Baumann
# Date: June 21 2008

# This program is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation; either version 2 of the License, or
# (at your option) any later version.

# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.

# You should have received a copy of the GNU General Public License
# along with this program; if not, write to the Free Software
# Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307 USA

# NOTE: this makefile relies upon the OpenCV libraries -lcv -lhighgui -lcvaux

# ==========================================================================

REPOS = /Users/mbaum/Code/Thunderbird/tbird_repos

INCDIR_MAC = -I. -I../../tbrprobe/tbrclient -I/usr/local/include/opencv -I/cs/public/src/opencv
INCDIR_LIN = -I. -I../../tbrprobe/tbrclient -I/usr/local/include/opencv 

INCDIR = -I$(REPOS)/vision/include/imageops

DBG    = #-g
OPT    = -O3
CPP    = g++
CFLAGS = $(DBG) $(OPT) $(INCDIR_MAC) $(INCDIR)

TBRCLIENTLIB = $(REPOS)/tbrprobe/tbrclient/tbrclient.a

LIBDIR_MAC = -L/usr/local/lib -L/cs/public/lib -L../../tbrprobe/tbrclient
LIBDIR_LIN = -L/usr/local/lib -L/cs/public/lib -L../../tbrprobe/tbrclient

LINK_MAC = -lm -lcv -lhighgui -lcvaux -lcxcore
LINK_LIN = -lm -lcv -lhighgui -lcvaux -lcxcore 

OBJ = 
LIBS = $(REPOS)/vision/lib/libimageops.a

PROGRAM = TemplateMatch

# ==========================================================================

.cpp.o:
	$(CPP) $(CFLAGS) -c $< -o $@

default: $(PROGRAM) example test

main.o: main.cpp

example.o: example.cpp

findMinima.o: findMinima.cpp

$(PROGRAM): main.o $(OBJ)
	$(CPP) $(CFLAGS) -o $(PROGRAM) main.o $(OBJ) $(LIBDIR_MAC) $(LIBS) $(LINK_MAC) $(TBRCLIENTLIB)

example: example.o
	$(CPP) $(CFLAGS) -o example example.o $(LIBDIR_MAC) $(LINK_MAC)

findMinima: findMinima.o
	$(CPP) $(CFLAGS) -o findMinima findMinima.o $(LIBDIR_MAC) $(LINK_MAC) $(LIBS)

test: test.o
	$(CPP) $(CFLAGS) -o test test.o $(OBJ) $(LIBDIR_MAC) $(LIBS) $(LINK_MAC)

clean:
	/bin/rm -f $(PROGRAM) *.o

clean-all: clean
	/bin/rm -f *~