#Master makefile for the vision code tree
#author: Matthew Baumann

#repository root path
REPOS = $(HOME)/snowbots

SRCROOT = $(REPOS)/src
LIBROOT = $(REPOS)/lib
INCROOT = $(REPOS)/include

#code tree path
VISIONSRC = $(REPOS)/vision/src
VISIONINC = $(REPOS)/vision/include
VISIONLIB = $(REPOS)/vision/lib

# Other dependencies
OPENCV_INCDIR = -I/usr/local/include/opencv -I/cs/public/src/opencv -I/usr/include/opencv
TBRCLIENT_HDR_DIR = $(REPOS)/tbrprobe/09/examples/cpp/tbrclient
TBRCLIENT_LIB_DIR = $(REPOS)/tbrprobe/09/examples/cpp/tbrclient

#ensure that the sub-makefiles can use the same macros
export REPOS VISIONSRC VISIONINC VISIONLIB OPENCV_INCDIR TBRCLIENT_HDR_DIR TBRCLIENT_LIB_DIR

#a list of all the code modules (in order)
MODULES = ConeFollow CourseFollow imageops guidance/colouravoid guidance/colourguidance guidance/localmap guidance/raceguidance

all: 
	for module in $(MODULES); do \
	$(MAKE) -C $(VISIONSRC)/$$module; \
	done;

clean:
	for module in $(MODULES); do \
	$(MAKE) -C $(VISIONSRC)/$$module clean; \
	done;

guidance/colouravoid: ConfigFile pid_throttle imageops
	$(MAKE) -C $(VISIONSRC)/$@

guidance/colourguidance: ConfigFile pid_throttle imageops
	$(MAKE) -C $(VISIONSRC)/$@

guidance/localmap: imageops
	$(MAKE) -C $(VISIONSRC)/$@

guidance/raceguidance: ConfigFile pid_throttle imageops
	$(MAKE) -C $(VISIONSRC)/$@

calibration/tbrcalibrate:
	$(MAKE) -C $(VISIONSRC)/$@

calibration/camcalibrate:
	$(MAKE) -C $(VISIONSRC)/$@

imageops:
	$(MAKE) -C $(VISIONSRC)/$@

pid_throttle:
	$(MAKE) -C $(VISIONSRC)/$@

ConfigFile:
	$(MAKE) -C $(VISIONSRC)/$@
