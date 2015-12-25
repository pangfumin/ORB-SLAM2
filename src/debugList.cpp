/*
 * debugList.cpp
 *
 *  Created on: Nov 16, 2015
 *      Author: sujiwo
 */


#include <vector>
#include <list>
#include <set>
#include <map>
#include <fstream>
#include <sstream>
#include "MapObjectSerialization.h"


using namespace std;


void debugList (map<idtype,int> &targetList, const char *filename)
{
	fstream fd;
	fd.open (filename, fstream::out);
	if (!fd.is_open())
		return;

	for (map<idtype,int>::iterator it=targetList.begin(); it!=targetList.end(); it++) {
		fd << it->first << " : " << it->second << endl;
	}

	fd.close();
}


