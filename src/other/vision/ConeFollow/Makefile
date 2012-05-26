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



INCDIR = -I. -I../../tbrprobe/tbrclient -I/usr/local/include/opencv -I/cs/public/src/opencv -I/Developer/SDKs/MacOSX10.5.sdk/System/Library/Frameworks/Python.framework/Versions/2.5/include/python2.5
DBG    = -g
OPT    = -O3
CPP    = g++
CFLAGS = $(DBG) $(OPT) $(INCDIR)
TBRCLIENTLIB = ../../tbrprobe/tbrclient/tbrclient.a
LIBDIR = -L/usr/local/lib #-L/cs/public/lib -L../../tbrprobe/tbrclient
LINK   = -lm -lcv -lhighgui -lcvaux -lcxcore -framework GLUT -framework OpenGL -framework Python
OBJ = imageOps.o util.o serialBridge.o
PROGRAM = ConeFollow

.cpp.o:
	$(CPP) $(CFLAGS) -c $< -o $@

default: $(PROGRAM)

imageOps.o: imageOps.cpp imageOps.h

util.o: util.cpp util.h

configReader.o: configReader.h configReader.cpp

serialBridge.o: serialBridge.h serialBridge.cpp

main.o: main.cpp

$(PROGRAM): main.o $(OBJ)
	$(CPP) $(CFLAGS) -o $(PROGRAM) main.o $(OBJ) $(LIBDIR) $(LINK) $(TBRCLIENTLIB)

configTest: configTest.o configReader.o
	$(CPP) $(CFLAGS) -o configTest configTest.o configReader.o

clean:
	/bin/rm -f $(PROGRAM) *.o

clean-all: clean
	/bin/rm -f *~