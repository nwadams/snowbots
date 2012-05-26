#!/bin/bash
rm tbrprobe_client.o
rm tbrprobe_client
rm quartz_drag_race.o quartz_drag_race quartz_drag_race.exe
gcc -c tbrprobe_client.c
gcc -c quartz_drag_race.c -I"/cygdrive/c/Progra~1/OpenCV/cxcore/include" -I"/cygdrive/c/Progra~1/OpenCV/cv/include" -I"/cygdrive/c/Progra~1/OpenCV/cvaux/include" -I"/cygdrive/c/Progra~1/OpenCV/ml/include" -I"/cygdrive/c/Progra~1/OpenCV/otherlibs/highgui"
g++ -static  -I"/cygdrive/c/Progra~1/OpenCV/cxcore/include" -I"/cygdrive/c/Progra~1/OpenCV/cv/include" -I"/cygdrive/c/Progra~1/OpenCV/cvaux/include" -I"/cygdrive/c/Progra~1/OpenCV/ml/include" -I"/cygdrive/c/Progra~1/OpenCV/otherlibs/highgui" -o quartz_drag_race.exe quartz_drag_race.o tbrprobe_client.o -L"/cygdrive/c/Progra~1/OpenCV/lib" -lhighgui -lml -lcvaux -lcv -lcxcore -lz -lavifil32 -lavicap32 -lvfw32 -lcomctl32 -ladvapi32 -lgdi32 -luser32 -lkernel32
