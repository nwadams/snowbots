/**
 * \file configReader.hpp
 * \author Matthew Baumann
 * \date 7-27-08
 */

#ifndef CONFIGREADER_HPP
#define CONFIGREADER_HPP

#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <sstream>

//#define CR_DEBUG

using namespace std;

typedef vector<string> sVec;

/**
 * A class for a reader that can parse vgraph configuration files
 */
class configReader{
	public:
	
	/// The Constructor
	configReader();
	
	/// The Destructor: ensures that the fstream is closed if the reader is deinstantiated
	~configReader();
	
	///Predicate to check if a file is currently open
	bool is_open();
	
	///Method to open a file in preparation for parsing
	bool open(char* filename);
	///Method to open a file in preparation for parsing
	bool open(string filename);
	
	///Method to close the file.
	bool close();
	
	///Method to extract lines from the configuration file as a vector of vectors of strings 
	/// \param prefix the string prefix of the desired type of line
	/// \return Returns a vector of vectors of strings.  Each vector of strings represents one line in the file that begins with prefix.
	vector<sVec> getEntry(string prefix);
	
	private:
	
	//the file stream that will manipulate the file
	fstream FS;
	
	
};

#endif
