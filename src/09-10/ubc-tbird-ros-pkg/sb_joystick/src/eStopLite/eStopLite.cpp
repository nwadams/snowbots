/*
 *	To Compile:
 *	"make clean" then "make", Prereq: 3 files (tbrprobe_client.c, tbrprobe_client.h and udp_constants.h)
 *
 *	To Modify:
 *		add your code to the loop in Step 3, you may want to adjust the sleep time for the loop depending on whether the program needs to be more sensitive to wiimote or not
 *
 *	outline:
 *	Step 1: connect with wiimote, to confirm the connection, every time "home" button is pressed, the wiimot double-rumbles, corresponding leds are lit up
 *			leds always represents the platform number on top of the netbook, this number needs to be checked to prevent confusion
 *			battery level is reported once wiimote is connected
 *	Step 2: start throttle 
 *	Step 3: start the forever loop (I am using a helloworld program)
 *		if any button other than B, power or home is press->permanent stop, wiimote rumbles
 *		if any button other than power or home is pressed in conjunction with B->temperary stop, robot gets back up when buntton is released, wiimote rumbles
 *		if home is pressed->wiimote double-rumbles
 *	Step 4: the program quits when wiimote gets out of the range or when "power" is pressed for more than 1 second
 *		leds go off
 *
 *	Specification:
 *		1. one wiimote is connected at a time
 *	Notes:
 *		1. there are two global pointers in helper.h
 *	Future plans, make this program a threaded library
 *	number the wiimotes, match up bluetooth address when connecting to a netbook
*/

#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <cwiid.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include "lookupTable.h"

#define FALSE 0
#define TRUE 1
#define THIS_MACHINE 4
#define NAP_TIME 200000 // in microsecond

// error callback
cwiid_err_t err;
cwiid_mesg_callback_t cwiid_callback;

// cwiid functions
void set_led_state(cwiid_wiimote_t *wiimote, unsigned char led_state);
void set_rpt_mode(cwiid_wiimote_t *wiimote, unsigned char rpt_mode);

// global pointers
cwiid_wiimote_t *wiimote;
struct cwiid_state *state;


// wiimote functions
void stateReport();
void setLEDs(int netbookNumber);
void setRumble(int rumble);
void doubleRumble();


int main (int argc, char *argv[])
{
	// ROS stuff	
	bool eStop = 0;	
	ros::init(argc, argv, "eStop");
	ros::NodeHandle n;
	ros::Publisher chatter_pub = n.advertise<std_msgs::Bool>("eStop", 100);
	ros::Rate loop_rate(1000000/NAP_TIME);

	// wiimote stuff
	struct cwiid_state stuff;
	state=&stuff;
	bdaddr_t bdaddr;
	unsigned int buttons;
	int action;;
	
	//not my code, ask cwiid if you have questions
	cwiid_set_err(err);

	// Connect to address given on command-line, if present
	if (argc > 1) {
		str2ba(argv[1], &bdaddr);
	}
	else {
		bdaddr = *BDADDR_ANY;
	}

	// Connect to the wiimote
	printf("[eStopLite] Put Wiimote in discoverable mode now (press 1+2)...\n");
	printf("[eStopLite] Press any key you when you are ready\n");
	getchar();
	if (!(wiimote = cwiid_open(&bdaddr, 0))) {
		fprintf(stderr, "[eStopLite] Unable to connect to wiimote\n");
		return -1;
	}
	if (cwiid_set_mesg_callback(wiimote, cwiid_callback)) {
		fprintf(stderr, "[eStopLite] Unable to set message callback\n");
	}

	//enable buttom messages, or information about the buttom will not be read.
	cwiid_command(wiimote,CWIID_CMD_RPT_MODE,CWIID_RPT_STATUS|CWIID_RPT_BTN);

	// confirm again
	if(cwiid_get_state(wiimote,&stuff))
		printf("[eStopLite] Can't find data!\n");

	// report wiimote state for the first time
	stateReport();	

	// let wii rumble, confirm connection with the user
	setLEDsBinary(THIS_MACHINE);
	setRumble(1);
	usleep(500000);
	setRumble(0);	

	// report batter usage
	while(stuff.battery==0)
		usleep(1000);
	printf("[eStopLite] wiimote battery: %d%% \n",(int)100.0*state->battery/CWIID_BATTERY_MAX);

	// report to user
	ROS_INFO("\n\nwiimote:\"I am alive!\"");

	while(ros::ok()) 
	{	
		// extract button info
		cwiid_get_state(wiimote, &stuff);
		buttons=state->buttons;
		
		// here is how I mapped out the buttons
		// you can set them all you want
		// well as long as you write a short manual like the one below
		// button map:
		// press any button to quit
		// press "home" for double-rumble confirmation, do not quit

		if(buttons & CWIID_BTN_2)	action=1;
		if(buttons & CWIID_BTN_1)	action=1;
		if(buttons & CWIID_BTN_A)	action=1;
		if(buttons & CWIID_BTN_MINUS)	action=1;
		if(buttons & CWIID_BTN_LEFT)	action=1;
		if(buttons & CWIID_BTN_RIGHT)	action=1;
		if(buttons & CWIID_BTN_DOWN)	action=1;
		if(buttons & CWIID_BTN_UP)	action=1;
		if(buttons & CWIID_BTN_PLUS)	action=1;

		if(buttons & CWIID_BTN_B)	action=1;
		if(buttons & CWIID_BTN_HOME)	action=2;
		
		switch(action)
		{
		case 1:
			eStop = TRUE;	
			std_msgs::Bool msg;
			msg.data = eStop;
			chatter_pub.publish(msg);
			ROS_INFO("wiimote:\"someone pressed a button, rumble\"\n");
			setRumble(TRUE);
			sleep(1);
			setRumble(FALSE);
		case 2:
			ROS_INFO("wiimote:\"someone pressed 'home' button, rumble rumble.\"\n");
			doubleRumble();
			break;
		default:
			break;
		}
		loop_rate.sleep();
	}
	return 0;
}

void stateReport()
{
	// led statues
	int netbookNumber=0;
	if(state->led & ESTOP_1) netbookNumber+=1;
	if(state->led & ESTOP_2) netbookNumber+=2;
	if(state->led & ESTOP_4) netbookNumber+=4;
	if(state->led & ESTOP_8) netbookNumber+=8;
	printf("wiimote is connected with netbook #%d \n", netbookNumber);
	// message enabled
	printf("Report Mode:");
	if (state->rpt_mode & CWIID_RPT_STATUS) printf(" STATUS");
	if (state->rpt_mode & CWIID_RPT_BTN) printf(" BTN");
	if (state->rpt_mode & CWIID_RPT_ACC) printf(" ACC");
	if (state->rpt_mode & CWIID_RPT_IR) printf(" IR");
	if (state->rpt_mode & CWIID_RPT_NUNCHUK) printf(" NUNCHUK");
	if (state->rpt_mode & CWIID_RPT_CLASSIC) printf(" CLASSIC");
	printf("\n");
	//battery
	printf("battery: %d%% \n",(int)100.0*state->battery/CWIID_BATTERY_MAX);
	//extention
	printf("extension: ");
	switch(state->ext_type)
	{
	case CWIID_EXT_NONE:
		printf("none \n");
		break;
	case CWIID_EXT_NUNCHUK:
		printf("nunchuck \n");
		break;
	case CWIID_EXT_CLASSIC:
		printf("classic \n");
		break;
	case CWIID_EXT_UNKNOWN:
		printf("not recognized \n");
		break;
	default:
		printf("Error! \n");
	}
	
	return;
}

void setLEDsBinary(int netbookNumber)
{
	unsigned char led_state=0;
	if(netbookNumber>=8){led_state+=ESTOP_8; netbookNumber-=8;}
	if(netbookNumber>=4){led_state+=ESTOP_4; netbookNumber-=4;}
	if(netbookNumber>=2){led_state+=ESTOP_2; netbookNumber-=2;}
	if(netbookNumber>=1){led_state+=ESTOP_1; netbookNumber-=1;}

	if (cwiid_set_led(wiimote, led_state)) {
		fprintf(stderr, "Error setting LEDs \n");
	}

	return;
}

void setRumble(int rumble)
{
	if (cwiid_set_rumble(wiimote, rumble)) {
		fprintf(stderr, "Error setting rumble\n");
	}
	return;
}

void doubleRumble()
{
	unsigned char rumble=TRUE;
	if (cwiid_set_rumble(wiimote, rumble)) {
		fprintf(stderr, "Error setting rumble\n");
	}
	usleep(100000);
	rumble=FALSE;
	if (cwiid_set_rumble(wiimote, rumble)) {
		fprintf(stderr, "Error setting rumble\n");
	}
	return;
}

void set_led_state(cwiid_wiimote_t *wiimote, unsigned char led_state)
{
	if (cwiid_set_led(wiimote, led_state)) {
		fprintf(stderr, "Error setting LEDs \n");
	}
}
	
void set_rpt_mode(cwiid_wiimote_t *wiimote, unsigned char rpt_mode)
{
	if (cwiid_set_rpt_mode(wiimote, rpt_mode)) {
		fprintf(stderr, "Error setting report mode\n");
	}
}

void err(cwiid_wiimote_t *wiimote, const char *s, va_list ap)
{
	if (wiimote) printf("%d:", cwiid_get_id(wiimote));
	else printf("-1:");
	vprintf(s, ap);
	printf("\n");
}

/* Prototype cwiid_callback with cwiid_callback_t, define it with the actual
 * type - this will cause a compile error (rather than some undefined bizarre
 * behavior) if cwiid_callback_t changes */
/* cwiid_mesg_callback_t has undergone a few changes lately, hopefully this
 * will be the last.  Some programs need to know which messages were received
 * simultaneously (e.g. for correlating accelerometer and IR data), and the
 * sequence number mechanism used previously proved cumbersome, so we just
 * pass an array of messages, all of which were received at the same time.
 * The id is to distinguish between multiple wiimotes using the same callback.
 * */
void cwiid_callback(cwiid_wiimote_t *wiimote, int mesg_count,
                    union cwiid_mesg mesg[], struct timespec *timestamp)
{
	int i, j;
	int valid_source;

	for (i=0; i < mesg_count; i++)
	{
		switch (mesg[i].type) {
		case CWIID_MESG_STATUS:
			printf("Status Report: battery=%d extension=",
			       mesg[i].status_mesg.battery);
			switch (mesg[i].status_mesg.ext_type) {
			case CWIID_EXT_NONE:
				printf("none");
				break;
			case CWIID_EXT_NUNCHUK:
				printf("Nunchuk");
				break;
			case CWIID_EXT_CLASSIC:
				printf("Classic Controller");
				break;
			default:
				printf("Unknown Extension");
				break;
			}
			printf("\n");
			break;
		case CWIID_MESG_BTN:
			printf("Button Report: %.4X\n", mesg[i].btn_mesg.buttons);
			break;
		case CWIID_MESG_ACC:
			printf("Acc Report: x=%d, y=%d, z=%d\n",
                   mesg[i].acc_mesg.acc[CWIID_X],
			       mesg[i].acc_mesg.acc[CWIID_Y],
			       mesg[i].acc_mesg.acc[CWIID_Z]);
			break;
		case CWIID_MESG_IR:
			printf("IR Report: ");
			valid_source = 0;
			for (j = 0; j < CWIID_IR_SRC_COUNT; j++) {
				if (mesg[i].ir_mesg.src[j].valid) {
					valid_source = 1;
					printf("(%d,%d) ", mesg[i].ir_mesg.src[j].pos[CWIID_X],
					                   mesg[i].ir_mesg.src[j].pos[CWIID_Y]);
				}
			}
			if (!valid_source) {
				printf("no sources detected");
			}
			printf("\n");
			break;
		case CWIID_MESG_NUNCHUK:
			printf("Nunchuk Report: btns=%.2X stick=(%d,%d) acc.x=%d acc.y=%d "
			       "acc.z=%d\n", mesg[i].nunchuk_mesg.buttons,
			       mesg[i].nunchuk_mesg.stick[CWIID_X],
			       mesg[i].nunchuk_mesg.stick[CWIID_Y],
			       mesg[i].nunchuk_mesg.acc[CWIID_X],
			       mesg[i].nunchuk_mesg.acc[CWIID_Y],
			       mesg[i].nunchuk_mesg.acc[CWIID_Z]);
			break;
		case CWIID_MESG_CLASSIC:
			printf("Classic Report: btns=%.4X l_stick=(%d,%d) r_stick=(%d,%d) "
			       "l=%d r=%d\n", mesg[i].classic_mesg.buttons,
			       mesg[i].classic_mesg.l_stick[CWIID_X],
			       mesg[i].classic_mesg.l_stick[CWIID_Y],
			       mesg[i].classic_mesg.r_stick[CWIID_X],
			       mesg[i].classic_mesg.r_stick[CWIID_Y],
			       mesg[i].classic_mesg.l, mesg[i].classic_mesg.r);
			break;
		case CWIID_MESG_ERROR:
			if (cwiid_close(wiimote)) {
				fprintf(stderr, "Error on wiimote disconnect\n");
				exit(-1);
			}
			exit(0);
			break;
		default:
			printf("Unknown Report");
			break;
		}
	}
}
