#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <iostream>
#include "wiimote.h"

using namespace std;
WiiDriver::WiiDriver()
{

}

bool WiiDriver::Init()
{
	//Wait for User to initiate Bluetooth Sync
	cout << "Press 1+2 on the Wiimote to start sync," << endl;
	cout << "then press ENTER to continue." << endl;
	getchar();
	
	
	//Attempt to Connect to wiimote
	cout << "Searching for Wiimote" << endl;
	remote = cwiid_connect(BDADDR_ANY,0);
	if( remote == NULL)
	{
		cout << "ERROR:Failed to Get Wiimote" << endl;
		return false;	
	}
	cout << "Wiimote Successfully Connected" <<endl;
	//This is where I choose what the wiimote sends for now status and buttons
	cwiid_command(remote,CWIID_CMD_RPT_MODE,CWIID_RPT_STATUS|CWIID_RPT_BTN);

	struct cwiid_state stuff;
	if(cwiid_get_state(remote,&stuff)){
		printf("[eStopLite] Can't find data!\n");
		return false;
	}

	printf("%i", stuff.buttons);

	cwiid_set_rumble(remote, 1);
	usleep(1000);
	cwiid_set_rumble(remote, 0);
	
	//If all goes well I win
	return true;
}


bool WiiDriver::IsPressed()
{
	//couple variables and their default state
	unsigned int buttons=0;
	struct cwiid_state state;
		
	//printf("poke the remote\n");
	//Poke the remote
	cwiid_get_state(remote,&state);

	//printf("extracting button\n");
	
	//Extract the button Int
	buttons=state.buttons;
	
	//printf("checking buttons\n");
	//If some buttons is pressed
	if( buttons != 0 ) {
		return true;
	} else {
		return false;
	}
}

WiiDriver::~WiiDriver()
{
	if(remote != NULL)
		cwiid_disconnect(remote);
}
