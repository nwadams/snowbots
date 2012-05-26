

# This Makefile relies on macros set by the master makefile:
#   REPOS - the path to the root folder of the repository
#   VISIONSRC, VISIONINC, VISIONLIB - the folders that contain the 
#     "vision" directory.
#   OPENCV_INCDIR - a list of -I include directives that contain possible
#     locations of the OpenCV library
#   TBRCLIENT_HDR_DIR, TBRCLIENT_LIB_DIR - header and lib locations of
#     the tbrclient library

DBG    = -g
OPT    = -O3
CPP    = g++

INCDIR = -I. $(OPENCV_INCDIR)
LIBDIR = -L/usr/local/lib -L/cs/public/lib 

LINK = -lm -lcv -lhighgui -lcvaux -lcxcore

CFLAGS = $(DBG) $(OPT) $(INCDIR)

CALIBRATOR = calib
CAPTURE = capture
TRANSFORM = birdseye

PROGRAMS = $(CALIBRATOR) $(TRANSFORM) $(CAPTURE)

# ==========================================================================

.cpp.o:
	$(CPP) $(CFLAGS) -c $< -o $@

default: $(PROGRAMS)

capture.o: capture.cpp

birdseye.o: birdseye.cpp

calib.o: calib.cpp

main.o: main.cpp

$(CALIBRATOR): calib.o
	$(CPP) $(CFLAGS) -o $(CALIBRATOR) calib.o $(LIBDIR) $(LINK)

$(CAPTURE): capture.o 
	$(CPP) $(CFLAGS) -o $(CAPTURE) capture.o $(LIBDIR) $(LINK)

$(TRANSFORM): birdseye.o 
	$(CPP) $(CFLAGS) -o $(TRANSFORM) birdseye.o $(LIBDIR) $(LINK)

clean:
	/bin/rm -f $(PROGRAMS) *.o

clean-all: clean
	/bin/rm -f *~
