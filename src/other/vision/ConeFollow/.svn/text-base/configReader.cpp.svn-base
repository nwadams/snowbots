/**
 * \file configReader.cpp
 * \author Matthew Baumann
 * \date 7-27-08
 */

#include "configReader.h"

using namespace std;

configReader::configReader(){
	#ifdef CR_DEBUG
		cout << "configReader instantiated" << endl;
	#endif

}

configReader::~configReader(){
	if(FS.is_open())
	{
		FS.close();
	}
	#ifdef CR_DEBUG
		cout << "configReader deinstantiated" << endl;
	#endif
}

bool configReader::is_open(){
	return FS.is_open();
}

bool configReader::open(char* filename)
{
	#ifdef CR_DEBUG
		cout << "configReader.open(" << filename << ")" << endl;
	#endif

	FS.open(filename,std::fstream::in);
	return FS.is_open();
}

bool configReader::open(string filename){
	FS.open(filename.c_str(),std::fstream::in);
	
		#ifdef CR_DEBUG
		cout << "configReader.open(" << filename << ")" << endl;
	#endif
	
	return FS.is_open();
}

bool configReader::close(){
	FS.close();
	return !FS.is_open();
}

vector<sVec> configReader::getEntry(string prefix){

	#ifdef CR_DEBUG
		cout << "configReader.getEntry(\"" << prefix << "\")" << endl;
	#endif

	//create the output:
	vector<sVec> lines;
	
	int line_num = 0;
	
	if(FS.is_open())
	{
		string line;
		
		//go through each line in the file using std::getLine
		while( getline( FS, line ) )
		{
			#ifdef CR_DEBUG
				//cout << "line " << line_num << " is: \"" << line << "\""<< endl;
			#endif
			
			line_num++;
			
			int entries = 0;
			std::stringstream SS;
			SS << line;
			bool enabled;
			
			string label;
			SS >> label;
			if(!SS.fail())
			{
				vector<string> line_vec;
			
				if(prefix.compare(label) == 0)
				{
					line_vec.push_back(label);
					string entry;
					SS >> entry;
					while(!SS.fail())
					{
						line_vec.push_back(entry);
						SS >> entry;
					}
					lines.push_back(line_vec);
				}
				
				
			}
		}
	}
	
	
	//set the fstream back to the beginning to allow the next query to search from the top
	FS.clear();              // forget we hit the end of file
	FS.seekg(0, ios::beg);   // move to the start of the file

	
	return lines;
}