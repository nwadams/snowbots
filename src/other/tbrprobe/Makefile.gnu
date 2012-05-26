# Makefile - to compile  

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
OBJLINK=tbrprobe.o pos_tracker.o pos_turret.o pos_optics.o sonar_prox.o pos_steer.o pos_battery.o ir_grade_axis.o raw_analog.o serial232.o furious_control.o motor_control.o nimu_control.o servo_control.o udp_thread.o
OBJ=tbrprobe.o interface/pos_tracker.o interface/pos_turret.o interface/pos_optics.o interface/raw_analog.o interface/sonar_prox.o interface/pos_steer.o interface/pos_battery.o interface/ir_grade_axis.o hardware/serial232.o hardware/furious_control.o hardware/motor_control.o hardware/nimu_control.o hardware/servo_control.o udp_thread.o
EXE= tbrprobe test_nimu vision_examples/ffilldemo vision_examples/astopdetect

default: all

all: $(EXE)

%.o : %.c
	$(CC) $(INCLUDE) $(CFLAGS) -c $<

tbrprobe: tbrprobe.o $(OBJ)
	@echo "Using robot config for $(ROBOT)"
	$(CC) $(CFLAGS) $(OBJLINK) -o $@ -lm
test_nimu: test_nimu.o nimu_control.o serial232.o
	$(CC) $(CFLAGS) $(OBJLINK) -o $@
ffilldemo: ffilldemo.o
	vision_examples/$(CC) $(LIB) $(CFLAGS) $^ -o $@ $(LDFLAGS)
astopdetect: astopdetect.o
	vision_examples/$(CC) $(LIB) $(CFLAGS) $^ -o $@ $(LDFLAGS)
.PHONY : clean
clean:
	rm -f *.o *.exe vision/*.o vision/*.exe hardware/*.o hardware/*.exe interface/*.o interface/*.exe $(EXE)
