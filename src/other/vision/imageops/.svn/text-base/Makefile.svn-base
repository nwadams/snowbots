
# Makefile for the imageops library
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

INCDIR = -I. $(OPENCV_INCDIR)

DBG    = -g
OPT    = -O3
CPP    = g++
LD     = g++
AR     = ar
CFLAGS = $(DBG) $(OPT) $(INCDIR)
LDFLAGS = $(LIBDIR) 

LIBDIR = -L/usr/local/lib -L/cs/public/lib -L$(TBRCLIENT_LIB_DIR)

LINK = -lm -lcv -lhighgui -lcvaux -lcxcore

OBJ = imageops.o util.o
LIBRARY = imageops.a

INCDEST = $(REPOS)/vision/include/imageops
LIBDEST = $(REPOS)/vision/lib
BINDEST = $(REPOS)/vision/bin

# ==========================================================================

.cpp.o:
	$(CPP) $(CFLAGS) -c $< -o $@

default: $(LIBRARY)

imageops.o: imageops.cpp imageops.h

util.o : util.cpp util.h


$(LIBRARY): $(OBJ)
	$(AR) rcs $(LIBRARY) $(OBJ)
	cp *.h $(INCDEST)
	cp $(LIBRARY) $(LIBDEST)

clean:
	/bin/rm -f $(LIBRARY) *.o

clean-all: clean
	/bin/rm -f *~
