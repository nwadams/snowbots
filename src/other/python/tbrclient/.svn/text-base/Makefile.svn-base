# Makefile for generating a Python interface to tbrclient.  See the 
# README file in this directory.

ifndef $(REPOS)
	# root directory of the snowbots repository
	REPOS = ../../..
endif

# install to snowbots/src/python/
INSTALL_DIR = ..

default: tbrclient.py

tbrclient_wrap.cpp: tbrclient.i
	swig -c++ -python -classic -o $@ $<

tbrclient_wrap.o: tbrclient_wrap.cpp
	g++ -fPIC -c -I$(REPOS)/include -I/usr/include/python2.5 -o $@ $<

tbrclient.py: tbrclient_wrap.o
	g++ -shared -o _tbrclient.so $< $(REPOS)/lib/tbrclient.a
	mv $@ $(INSTALL_DIR)/
	mv _tbrclient.so $(INSTALL_DIR)/

clean:
	rm -rf build/ cpp/ tbrclient_wrap.* tbrclient.py
	rm -f $(INSTALL_DIR)/tbrclient.py $(INSTALL_DIR)/tbrclient.pyc $(INSTALL_DIR)/_tbrclient.so
