# Makefile - to compile tbrprobe
# Author: Ash McKay
# Modifications: Ian Phillips

SNOWBOTS_ROOT = $(realpath ../..)
SNOWBOTS_SRC = $(SNOWBOTS_ROOT)/src
SNOWBOTS_LIB = $(SNOWBOTS_ROOT)/lib
SNOWBOTS_INC = $(SNOWBOTS_ROOT)/include
SNOWBOTS_BIN = $(SNOWBOTS_ROOT)/bin

CC=gcc

# Using default robot config traxxas
# Use make 'ROBOT=traxxas|duratrax|snowfury'

ifndef $(ROBOT)
	ROBOT=traxxas
endif


INCLUDE= -I/usr/local/include/opencv
CFLAGS= -Wall -g -pthread -D$(ROBOT)
LDFLAGS= -lm -lcv -lcxcore -lhighgui
LIB= -L/usr/local/lib
OBJLINK=tbrprobe.o pos_tracker.o pos_turret.o sonar_prox.o pos_optics.o pos_steer.o pos_battery.o ir_grade_axis.o serial232.o raw_analog.o furious_control.o motor_control.o nimu_control.o servo_control.o udp_thread.o
OBJ=tbrprobe.o interface/pos_tracker.o interface/pos_optics.o interface/pos_turret.o interface/sonar_prox.o interface/pos_steer.o  interface/raw_analog.o interface/pos_battery.o interface/ir_grade_axis.o hardware/serial232.o hardware/furious_control.o hardware/motor_control.o hardware/nimu_control.o hardware/servo_control.o udp_thread.o
EXE= tbrprobe test_nimu

INCDEST=$(SNOWBOTS_INC)/tbrprobe
BINDEST=$(SNOWBOTS_BIN)

default: all

all: $(EXE)

%.o : %.c
	$(CC) $(INCLUDE) $(CFLAGS) -c $<

tbrprobe: tbrprobe.o $(OBJ)
	@echo "Using robot config for $(ROBOT)"
	$(CC) -lm $(CFLAGS) $(OBJLINK) -o $@
	cp tbrprobe $(BINDEST)/
	cp udp_constants.h $(INCDEST)/

test_nimu: test_nimu.o nimu_control.o serial232.o
	$(CC) -lm $(CFLAGS) $(OBJLINK) -o $@

.PHONY : clean
clean:
	rm -f *.o *.exe hardware/*.o hardware/*.exe interface/*.o interface/*.exe $(EXE)
	rm -f $(INCDEST)/udp_constants.h
	rm -f $(BINDEST)/tbrprobe
