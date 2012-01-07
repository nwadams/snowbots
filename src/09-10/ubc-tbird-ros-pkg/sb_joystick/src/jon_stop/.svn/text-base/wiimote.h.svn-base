// Abstracted interface for wiimote control 
//#include <cwiid/cwiid.h>
#include <cwiid.h>

#ifndef WIIMOTE_H
#define WIIMOTE_H


class WiiDriver
{
	public:	
		WiiDriver();
		~WiiDriver();
		bool Init();
		bool IsPressed();	
	
	private:
		cwiid_wiimote_t* remote; //remote pointer for wiimote
};

#endif

