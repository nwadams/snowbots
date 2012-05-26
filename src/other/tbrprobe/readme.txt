======================================
How to compile:
To use different robot configuration files, 
there are options for the makefile in the form of:

make 'ROBOT=[robot_config_name]'

where robot_config_name can be traxxas, duratrax or snowfury.
For example the quartz platform is compiled as:

make -f Makefile ROBOT=duratrax

Note: If the ROBOT variable is not defined (just typing 'make') 
         it will be compiled for traxxas by default.

======================================
Rules of thumb:
* keep the HAL code libs strict ANSI C (GCC)
* keep the interface simple 
* Keep up to date application binary in the bin/os/ path

Standards:
* Do not Modify UDP enumerated message formats 
   (search, then add more... if not documented they may be removed.)
* Do not change robot calibrations 
   (add your own configuration area and unique robot name)
* Do not modify Hardware interface key signatures
   (add your own class of device that follows the standard)
* Do not be tempted to move AI code etc... into this device layer
* This is 100% GPL code


Works on most tested platforms:
* Microsoft Windows XP
* Debian Linux (Ubuntu 7.10/8.4, knoppix)
* Apple MacOS 10.5

Suggested AI frameworks to connect:
CARMEN
Player/Stage & Gazebo & SSPP


======================================
Contributing Authors:

Ashley Mckay

Ian Phillips

Nicolas Saunier
