CC = g++
REPOS = $(HOME)/tbird
SRC = $(REPOS)/vision/src
TBRCLIENT = $(REPOS)/tbrprobe/09/examples/cpp/tbrclient

INCLUDE = -I$(REPOS)/vision/include -I$(TBRCLIENT) -I/usr/include/opencv
LIBDIR = -L$(REPOS)/vision/lib -L$(TBRCLIENT)
LINK = $(TBRCLIENT)/tbrclient.a -lhighgui

APP = coursemap
OBJS = main.o course_map.o

default: $(APP)

$(APP): $(OBJS)
	$(CC) -o $@ $(OBJS) $(LIBDIR) $(LINK)

%.o: %.cpp
	$(CC) $(INCLUDE) -c $<


clean:
	rm -f $(APP) $(OBJS)
