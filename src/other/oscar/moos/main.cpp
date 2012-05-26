#include "FuriousInstrument.h"

int main()
{
	const char *missionFile = "mission.moos";
	const char *appName = "FuriousInstrument";

	FuriousInstrument furious;

	furious.Run(appName, missionFile);

	return 0;
}
