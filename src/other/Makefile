#Master makefile for snowbots/src/
#author: Matthew Baumann & Ian Phillips

#a list of all the code modules (in order)
MODULES = tbrprobe tbrclient

.PHONY: all clean $(MODULES)
all: $(MODULES)

tbrprobe:
	$(MAKE) -C $@

tbrclient: tbrprobe
	$(MAKE) -C $@

clean:
	for module in $(MODULES); do \
	$(MAKE) -C $$module clean; \
	done;

