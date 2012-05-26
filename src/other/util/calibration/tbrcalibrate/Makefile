# Makefile for the colour based visual guidance program
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

# This Makefile relies on macros set by the master makefile:
#   REPOS - the path to the root folder of the repository
#   VISIONSRC, VISIONINC, VISIONLIB - the folders that contain the 
#     "vision" directory.
#   OPENCV_INCDIR - a list of -I include directives that contain possible
#     locations of the OpenCV library
#   TBRCLIENT_HDR_DIR, TBRCLIENT_LIB_DIR - header and lib locations of
#     the tbrclient library

ifndef $(REPOS)
	REPOS = $(HOME)/snowbots
endif

INCDIR = $(REPOS)/include
LIBDIR = $(REPOS)/lib
BINDEST = $(REPOS)/bin

DBG    = -g
OPT    = -O3
CPP    = g++

INCLIST = -I. $(OPENCV_INCDIR) -I$(INCDIR)
LIBLIST = -L/usr/local/lib -L/cs/public/lib -L$(LIBDIR)

LINK = $(LIBDIR)/tbrclient.a -lm -lcv -lhighgui -lcvaux -lcxcore
OBJ =  main.o 

CFLAGS = $(DBG) $(OPT) $(INCLIST)

PROGRAM = tbrcalibrate

# ==========================================================================

.cpp.o:
	$(CPP) $(CFLAGS) -c $< -o $@

default: $(PROGRAM)

#main.o: main.cpp $(TBRCLIENT_HDR_DIR)/tbrclient.h

$(PROGRAM): $(OBJ) 
	$(CPP) $(CFLAGS) -o $(PROGRAM) $(OBJ) $(LIBLIST) $(LINK)
	cp $(PROGRAM) $(BINDEST)/

clean:
	/bin/rm -f $(PROGRAM) *.o
	rm -i $(BINDEST)/$(PROGRAM)

clean-all: clean
	/bin/rm -f *~
