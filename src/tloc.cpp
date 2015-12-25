/*
 * tloc.cpp
 *
 *  Created on: Oct 15, 2015
 *      Author: sujiwo
 */


#include <iostream>
#include <sstream>
#include <fstream>
#include <string>
#include <vector>
#include <map>
#include <cmath>
#include <cstring>

#include <boost/archive/binary_oarchive.hpp>
#include <boost/archive/binary_iarchive.hpp>
#include <boost/serialization/split_free.hpp>
#include <boost/serialization/map.hpp>
#include <boost/serialization/vector.hpp>
#include <boost/serialization/list.hpp>


using namespace std;


class Whatizit: public map<unsigned int, double>
{
public:
	Whatizit () {}
};


BOOST_SERIALIZATION_SPLIT_FREE (Whatizit);


//int main (int argc, char *argv[])
//{
//	Whatizit w1;
//
//	w1[0] = M_PI;
//	w1[3] = 1e2;
//
//	stringstream outs;
//	boost::archive::binary_oarchive outarc (outs);
//
//	outarc << w1;
//	string ms = outs.str();
//
//	Whatizit w2;
//	string msx (ms.c_str(), ms.size());
//	stringstream instr (msx);
//	boost::archive::binary_iarchive inarc (instr);
//	inarc >> w2;
//
//	cout << w2[0] << endl;
//	cout << w2[3] << endl;
//
//	return 0;
//}

/*
 * This code shows that serialization can be performed piecemeal,
 * but deserialization must be done in exact order as serialization
 */

typedef long unsigned int luint;


void debugVectorRecords (const vector<list<luint> > &records)
{
	for (int i=0; i<records.size(); i++) {
		cout << i << ": ";
		for (list<luint>::const_iterator it=records[i].begin(); it!=records[i].end(); ++it) {
			cout << *it << ", ";
		}
		cout << endl;
	}
}


void debugMapRecords (const map<luint,string> &records)
{
	for (map<luint,string>::const_iterator it=records.begin(); it!=records.end(); ++it) {
		const luint &id = it->first;
		const string &str = it->second;
		cout << id << ": " << str << endl;
	}
}


void createRecord ()
{
	vector <list<luint> > invertedList;
	map <luint, string> stringList;

	luint l1[] = {99, 0, 5, 8};
	list<luint> mlist1 (l1, l1+sizeof(l1)/sizeof(luint));
	invertedList.push_back(mlist1);

	luint l2[] = {6, 3, 12, 100, 114};
	list<luint> mlist2 (l2, l2+sizeof(l2)/sizeof(luint));
	invertedList.push_back(mlist2);

	stringList[0] = "Hello 1";
	stringList[5] = "We are in Japan";

	debugVectorRecords (invertedList);
	debugMapRecords (stringList);

	fstream fs;
	fs.open ("/tmp/test.txt", fstream::out|fstream::in|fstream::app);
	if (fs.fail())
		throw "unable to create";
	stringstream ss;
	boost::archive::binary_oarchive outarc (ss);
	outarc << invertedList;
	outarc << stringList;
	string ms = ss.str();
	fs.write (ms.data(), ms.size());

	fs.close();
}


void retrieveAllRecord ()
{
	vector<list<luint> > mInvertedFile;
	map<luint,string> mStrList;

	fstream fx;
	fx.open ("/tmp/test.txt", fstream::in);
	boost::archive::binary_iarchive inarc (fx);
	inarc >> mInvertedFile;
	inarc >> mStrList;

	debugMapRecords (mStrList);
	debugVectorRecords (mInvertedFile);

	fx.close();
}


int main (int argc, char *argv[])
{
	if (argc == 1)
		createRecord();
	else if (strcmp(argv[1], "1")==0)

	return 0;
}
