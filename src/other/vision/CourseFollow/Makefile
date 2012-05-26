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

TBRPROBE = ../../tbrprobe/09
TBRCLIENT = $(TBRPROBE)/examples/cpp/tbrclient
TBRCLIENTLIB = $(TBRCLIENT)/tbrclient.a

INCDIR_MAC = -I. -I$(TBRCLIENT) -I/usr/local/include/opencv -I/cs/public/src/opencv
INCDIR_LIN = -I. -I$(TBRCLIENT) -I/usr/local/include/opencv -I/usr/include/opencv -I/usr/include/python2.5

DBG    = -g
OPT    = -O3
CPP    = g++
CFLAGS = $(DBG) $(OPT) $(INCDIR_LIN)

LIBDIR_MAC = -L/usr/local/lib -L/cs/public/lib -L$(TBRCLIENT)
LIBDIR_LIN = -L/usr/local/lib -L/cs/public/lib -L$(TBRCLIENT) -L/usr/lib/python2.5

LINK_MAC = -lm -lcv -lhighgui -lcvaux -lcxcore
LINK_LIN = -lm -lcv -lhighgui -lcvaux -lcxcore 

OBJ = imageOps.o util.o configReader.o

PROGRAM = CourseFollow

# ==========================================================================

.cpp.o:
	$(CPP) $(CFLAGS) -c $< -o $@

default: $(PROGRAM)

imageOps.o: imageOps.cpp imageOps.h

util.o: util.cpp util.h

configReader.o: configReader.cpp configReader.cpp

main.o: main.cpp

$(PROGRAM): main.o $(OBJ)
	$(CPP) $(CFLAGS) -o $(PROGRAM) main.o $(OBJ) $(LIBDIR_LIN) $(LINK_LIN) $(TBRCLIENTLIB)

clean:
	/bin/rm -f $(PROGRAM) *.o

clean-all: clean
	/bin/rm -f *~
