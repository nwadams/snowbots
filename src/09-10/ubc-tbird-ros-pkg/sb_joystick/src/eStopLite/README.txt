A Tutorial on Controlling Snowbots with Wiimote
============================================================

Some wiimote code: we have two versions

===========================
Koko's code:
routine:
1. [basic] declare a pointer cwiid_wiimote_t, which is conventionally called 'wiimote'. No need to point it to any memory, because a function (later) will communicate with wiimote and find the data buffer in memory. (for some suggestion on naming convention, read note 1)
2. [basic] declare a cwiid_state. The naming convention is 'state'. Reports on button events, accelerometer reading, extension report, etc will be stored here, but then you need to enable these functions manually. (see note 3 for reason) This time you must allocate memory.
3. [basic] declare a bdaddr_t called 'bdaddr', and other variables you may need.
4. [advanced] enable error reports, so that you will know the reason what things go wrong
5. [basic] point 'wiimote' pointer to data buffer in memory. use 'wiimote = cwiid_open(&bdaddr,0)' to open
[advance] you may wish to make sure that 'wiimote' doesn't point to 0. 'if(!(wiimote = cwiid_open(&bdaddr,0)))' and print error will do the trick.
6. [advanced] connect to message callback function.
7. [basic] 'cwiid_request_status(wiimote)' is essential. (Haven't figure out what it is for, but you will see button data if you don't call this, I think.)
8. [basic] To enable reports on button events, you need 'set_rpt_mode(wiimote,CWIID_RPT_BTN)'. To enable both accelerometer report and button report you need 'set_rpt_mode(wiimote,CWIID_RPT_BTN|CWIID_RPT_ACC)'. You may wish to declare a variable of type 'unsigned char' to store the report mode (good coding style). Naming convention for this variable is 'rpt_mode'.
9. [costumized] almost ready to use wiimote for controlling the robot. This is your last chance to call the  setup routine for the robot. (See note 4 on why we set up robot after connecting to wiimote)
10. [basic] call 'cwiid_get_state(wiimote, &state)' to get update on event button
11.[basic] finally, you can access 'state' and the button event will be stored there. Accelerometer info will be there, too if you turned on the function in step 8.
12. [costumized] make different buttons trigger some different events
13. [basic] you need to call 'cwiid_get_state(wiimote, &state)' every time before reading 'state', else you will be reading the same old data.

note from Koko
1. the advantage of following naming convention is that code written by different people is interchangable without having to change all variables' name.
Also, you may wish to declare global pointers so that all functions have access to the same wiimote
2. advanced steps are mostly for security reasons. For instance, if the connect goes wrong, cwiid can report to you roughly the cause of failure.
3. only enable functions that you need. Program can potentially run faster
4. you only want to drive your robot when wiinote eStop is safely in place.
5. let Koko know if you want to use wiimote not only as  an eStop. She is happy to donate some code.

===========================
Jon's Code: 

Jonathon Fraser wrote a class in c++. It was extensively used last year to drive the robot, so it should be fairly robust.
Instruction on how to use this code is coming soon.