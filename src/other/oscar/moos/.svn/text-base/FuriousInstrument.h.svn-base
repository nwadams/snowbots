#ifndef FURIOUSINSTRUMENT_H
#define FURIOUSINSTRUMENT_H

#include <MOOSLIB/MOOSInstrument.h>

class FuriousInstrument : public CMOOSInstrument
{
protected:
	bool OnNewMail(MOOSMSG_LIST &newMail);
	bool Iterate();
	bool OnConnectToServer();
	bool InitialiseSensor();
private:
	char command_buf[100];
	int servos[] = {0, 0, 0, 0, 0, 0, 0, 0};
	int unused[] = {0, 0, 0, 0};
	int sonar    = 0;
};
#endif
