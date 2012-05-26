INTRODUCTION

This directory contains a driver for the Furious microcontroller module.


COMPARISON WITH TBRPROBE

The files in this directory (src/c/furious) constitute a "lightweight" 
alternative to src/c/tbrprobe.  Tbrprobe is the "industrial-strength" version, 
designed for maximum performance and cross-platform compatibility, and is
multi-threaded to support multiple simultaneous devices.  This driver
trades off those benefits in favour of simple, readable code and a function-
oriented, modular design.  Consider this the "training-wheels" version.


DIRECTORY CONTENTS

The files serial232.c and serial232.h implement the serial communications 
layer.

furious_driver.* implement the low-level interface to the board.
Values sent to and returned from these functions are the raw values (eg,
PPM signal and analog-to-digital) that the Furious board deals with directly.

