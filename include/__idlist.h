/*
 * idlist.h
 *
 *  Created on: Nov 15, 2015
 *      Author: sujiwo
 */

#ifndef INCLUDE___IDLIST_H_
#define INCLUDE___IDLIST_H_


#include <set>
#include <vector>
#include <list>
#include <map>


using std::set;
using std::vector;
using std::map;
using std::list;

typedef unsigned long int idtype;


template<typename MapObject>
set<idtype> createIdList(const set<MapObject*> &mapObjectList)
{
	typedef typename set<MapObject*>::const_iterator ciMapObj;
	set<idtype> idList;

	for (ciMapObj it=mapObjectList.begin(); it!=mapObjectList.end(); it++) {
		MapObject *obj = *it;
		idList.insert (obj->mnId);
	}

	return idList;
}


template<typename MapObject>
set<MapObject*> createObjectList (const set<idtype> &mapIdList)
{
	set<MapObject*> objSet;

	for (set<idtype>::const_iterator it=mapIdList.begin(); it!=mapIdList.end(); it++) {
		MapObject *obj = MapObject::objectListLookup [*it];
		objSet.insert (obj);
	}

	return objSet;
}


template<typename MapObject>
vector<idtype> createIdList(const vector<MapObject*> &mapObjectList)
{
	vector<idtype> idList;

	for (int i=0; i<mapObjectList.size(); i++) {
		MapObject *obj = mapObjectList[i];
		idList.push_back (obj->mnId);
	}

	return idList;
}


template<typename MapObject>
vector<MapObject*> createObjectList (const vector<idtype> &mapIdList)
{
	vector<MapObject*> objVect;

	for (vector<idtype>::const_iterator it=mapIdList.begin(); it!=mapIdList.end(); it++) {
		MapObject *obj = MapObject::objectListLookup [*it];
		objVect.push_back (obj);
	}

	return objVect;
}


template<typename MapObject>
list<idtype> createIdList(const list<MapObject*> &mapObjectList)
{
	typedef typename list<MapObject*>::const_iterator ciMapObj;
	list<idtype> idList;

	for (ciMapObj it=mapObjectList.begin(); it!=mapObjectList.end(); it++) {
		MapObject *obj = *it;
		idList.push_back (obj->mnId);
	}

	return idList;
}


template<typename MapObject>
list<MapObject*> createObjectList (const list<idtype> &mapIdList)
{
	list<MapObject*> objVect;

	for (list<idtype>::const_iterator it=mapIdList.begin(); it!=mapIdList.end(); it++) {
		MapObject *obj = MapObject::objectListLookup [*it];
		objVect.push_back (obj);
	}

	return objVect;
}


template<typename MapObject, typename W>
map<idtype, W> createIdList(const map<MapObject*, W> &mapObjectList)
{
	typedef typename map<MapObject*, W>::const_iterator ciMapObj;
	map<idtype, W> idList;

	for (ciMapObj it=mapObjectList.begin(); it!=mapObjectList.end(); it++) {
		MapObject *obj = it->first;
		idList[obj->mnId] = it->second;
	}

	return idList;
}


template<typename MapObject, typename W>
map<MapObject*, W> createObjectList (const map<idtype, W> &mapIdList)
{
	map<MapObject*, W> objMap;
	typedef typename map<idtype, W>::const_iterator ci_t;

	for (ci_t it=mapIdList.begin(); it!=mapIdList.end(); it++) {
		MapObject *obj = MapObject::objectListLookup [it->first];
		objMap [obj] = it->second;
	}
	return objMap;
}


#endif /* INCLUDE___IDLIST_H_ */
