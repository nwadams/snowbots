TBRPROBE_BASE_DIR=../../../
PYTHON_PATH=/usr/include/python2.5/

default: _tbrclient.so

tbrclient_wrap.c: tbrclient.i
	swig -python -o $@ $<

tbrclient.o: tbrclient.c
	gcc -c -fPIC -I$(TBRPROBE_BASE_DIR) tbrclient.c

tbrclient_wrap.o: tbrclient_wrap.c
	gcc -c -fPIC -I$(PYTHON_PATH) -o $@ $<

_tbrclient.so: tbrclient.o tbrclient_wrap.o
	gcc -shared -o $@ $?

tbrdiag: tbrdiag.c tbrclient.c
	gcc -I$(TBRPROBE_BASE_DIR) -o $@ $?

clean:
	rm -f *.o *.a *.so tbrclient_wrap.c tbrclient.py* tbrdiag
