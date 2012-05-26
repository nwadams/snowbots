/**
 * \file configTest.cpp
 * \author Matthew Baumann
 * \date 7-27-08
 * Test Driver to unit test the configReader class
 */

#include <iostream>
#include "configReader.h"

using namespace std;

void printCrOutput(vector<sVec> lines);

int main(int argc, char* argv[])
{
	if(argc < 2)
	{
		cout << "Error: expected argument: <config file name>" << endl;
		return 1;
	}

	configReader CR;
	if(!CR.open(argv[1]))
	{
		cout << "Error: file \"" << argv[1] << "\" could not be opened." << endl;
		return 1;
	}
	
	printCrOutput(CR.getEntry("bogusprefix"));
	printCrOutput(CR.getEntry("HoughParams"));
	printCrOutput(CR.getEntry("Resolution"));
	
	
	return 0;
}

void printCrOutput(vector<sVec> lines)
{
	for(int i = 0; i < lines.size(); i++)
	{
		cout << "Entry " << i << ": ";
		for(int j = 0; j < lines[i].size(); j++)
		{
			cout << "\"" << lines[i][j] << "\"\t";
		}
		cout << endl;
	}
}