/**
* This file is part of ORB-SLAM.
*
* Copyright (C) 2014 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <http://webdiis.unizar.es/~raulmur/orbslam/>
*
* ORB-SLAM is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM. If not, see <http://www.gnu.org/licenses/>.
*/

#include "Map.h"
#include "KeyFrame.h"
#include "MapPoint.h"
#include "KeyFrameDatabase.h"
#include <cstdio>
#include <exception>
#include <string>
#include <algorithm>

#include <boost/archive/binary_oarchive.hpp>
#include <boost/archive/binary_iarchive.hpp>
#include "KeyFrame.h"
#include "MapPoint.h"
#include "MapObjectSerialization.h"


using std::string;


#define MapOctreeResolution 1.0


namespace ORB_SLAM
{

Map::Map() :
	MapIsReadonly (false)
{
    mbMapUpdated= false;
    mnMaxKFid = 0;
}

void Map::AddKeyFrame(KeyFrame *pKF)
{
    boost::mutex::scoped_lock lock(mMutexMap);
    mspKeyFrames.insert(pKF);
    if(pKF->mnId>mnMaxKFid)
        mnMaxKFid=pKF->mnId;
    mbMapUpdated=true;
}

void Map::AddMapPoint(MapPoint *pMP)
{
    boost::mutex::scoped_lock lock(mMutexMap);
    mspMapPoints.insert(pMP);
    mbMapUpdated=true;
}

void Map::EraseMapPoint(MapPoint *pMP)
{
    boost::mutex::scoped_lock lock(mMutexMap);
    mspMapPoints.erase(pMP);
    mbMapUpdated=true;
}

void Map::EraseKeyFrame(KeyFrame *pKF)
{
    boost::mutex::scoped_lock lock(mMutexMap);
    mspKeyFrames.erase(pKF);
    mbMapUpdated=true;
}

void Map::SetReferenceMapPoints(const vector<MapPoint *> &vpMPs)
{
    boost::mutex::scoped_lock lock(mMutexMap);
    mvpReferenceMapPoints = vpMPs;
    mbMapUpdated=true;
}

vector<KeyFrame*> Map::GetAllKeyFrames()
{
    boost::mutex::scoped_lock lock(mMutexMap);
    return vector<KeyFrame*>(mspKeyFrames.begin(),mspKeyFrames.end());
}

vector<MapPoint*> Map::GetAllMapPoints()
{
    boost::mutex::scoped_lock lock(mMutexMap);
    return vector<MapPoint*>(mspMapPoints.begin(),mspMapPoints.end());
}

int Map::MapPointsInMap()
{
    boost::mutex::scoped_lock lock(mMutexMap);
    return mspMapPoints.size();
}

int Map::KeyFramesInMap()
{
    boost::mutex::scoped_lock lock(mMutexMap);
    return mspKeyFrames.size();
}

vector<MapPoint*> Map::GetReferenceMapPoints()
{
    boost::mutex::scoped_lock lock(mMutexMap);
    return mvpReferenceMapPoints;
}

bool Map::isMapUpdated()
{
    boost::mutex::scoped_lock lock(mMutexMap);
    return mbMapUpdated;
}

void Map::SetFlagAfterBA()
{
    boost::mutex::scoped_lock lock(mMutexMap);
    mbMapUpdated=true;

}

void Map::ResetUpdated()
{
    boost::mutex::scoped_lock lock(mMutexMap);
    mbMapUpdated=false;
}

unsigned int Map::GetMaxKFid()
{
    boost::mutex::scoped_lock lock(mMutexMap);
    return mnMaxKFid;
}

void Map::clear()
{
    for(set<MapPoint*>::iterator sit=mspMapPoints.begin(), send=mspMapPoints.end(); sit!=send; sit++)
        delete *sit;

    for(set<KeyFrame*>::iterator sit=mspKeyFrames.begin(), send=mspKeyFrames.end(); sit!=send; sit++)
        delete *sit;

    mspMapPoints.clear();
    mspKeyFrames.clear();
    mnMaxKFid = 0;
    mvpReferenceMapPoints.clear();
}


const char *signature = "ORBSLAM";

void Map::createMapFileHeader (MapFileHeader &header)
{
	memcpy (header.signature, signature, sizeof(header.signature));
	header.numOfKeyFrame = this->mspKeyFrames.size();
	header.numOfMapPoint = this->mspMapPoints.size();
	header.numOfReferencePoint = this->mvpReferenceMapPoints.size();
}


void Map::saveToDisk (const string &filename, KeyFrameDatabase *keyframeDatabase)
{
	MapFileHeader header;
	createMapFileHeader (header);

	/*
	 * See notes.txt for explanation of map serialization order
	 */

	fstream mapFileFd;
	mapFileFd.open (filename.c_str(), fstream::out | fstream::trunc);
	if (!mapFileFd.is_open())
		throw MapFileException();
	mapFileFd.write ((const char*)&header, sizeof(header));
	cout << "Header written: " << header.numOfKeyFrame << " keyframes, " << header.numOfMapPoint << " points" << endl;

	boost::archive::binary_oarchive mapArchive (mapFileFd);

	int p = 0;
	for (set<KeyFrame*>::const_iterator kfit=mspKeyFrames.begin(); kfit!=mspKeyFrames.end(); kfit++) {
		const KeyFrame *kf = *kfit;
		if (kf==NULL) {
			cerr << endl << "NULL KF found" << endl;
			continue;
		}
		mapArchive << *kf;
		cout << "Keyframes: " << ++p << "/" << mspKeyFrames.size() << "\r";
	}
	cout << endl;

	for (set<MapPoint*>::iterator mpit=mspMapPoints.begin(); mpit!=mspMapPoints.end(); mpit++) {
		const MapPoint *mp = *mpit;
		if (mp==NULL) {
			cerr << endl << "NULL MP found" << endl;
			continue;
		}
		mapArchive << *mp;
	}

	vector<idtype> vmvpReferenceMapPoints = createIdList (mvpReferenceMapPoints);
	mapArchive << vmvpReferenceMapPoints;

	mapArchive << *keyframeDatabase;

	mapFileFd.close();
}


struct _keyframeTimestampSortComparator {
	bool operator() (KeyFrame *kf1, KeyFrame *kf2)
	{ return kf1->mTimeStamp < kf2->mTimeStamp; }
} keyframeTimestampSortComparator;


void Map::loadFromDisk(const string &filename, KeyFrameDatabase *kfMemDb)
{
	MapFileHeader header;

	fstream mapFileFd;
	mapFileFd.open (filename.c_str(), fstream::in);
	if (!mapFileFd.is_open())
		throw BadMapFile();
	mapFileFd.read ((char*)&header, sizeof(header));

	if (strcmp(header.signature, signature) !=0)
		throw BadMapFile();
//	cout << "Keyframes: " << header.numOfKeyFrame << ", MapPoint: " << header.numOfMapPoint << endl;

	boost::archive::binary_iarchive mapArchive (mapFileFd);

	for (int p=0; p<header.numOfKeyFrame; p++) {
		KeyFrame *kf = new KeyFrame;
		mapArchive >> *kf;
		if (!kf->isBad())
			mspKeyFrames.insert (kf);
		kfListSorted.push_back(kf);
		// XXX: Need to increase KeyFrame::nNextId
		// Also, adjust Frame::nNextId
		KeyFrame::nNextId = kf->mnId + 1;
		Frame::nNextId = kf->mnId + 2;
	}

	std::sort(kfListSorted.begin(), kfListSorted.end(), keyframeTimestampSortComparator);
	for (int p=0; p<kfListSorted.size(); p++) {
		KeyFrame *kf = kfListSorted[p];
		kfMapSortedId[kf] = p;
	}

	for (int p=0; p<header.numOfMapPoint; p++) {
		MapPoint *mp = new MapPoint;
		mapArchive >> *mp;
		mspMapPoints.insert (mp);
		// XXX: Need to increase MapPoint::nNextId
		MapPoint::nNextId = mp->mnId + 1;
	}

	if (kfMemDb==NULL)
		return;

	for (set<KeyFrame*>::iterator kfset=mspKeyFrames.begin(); kfset!=mspKeyFrames.end(); kfset++) {
		(*kfset)->fixConnections (this, kfMemDb);
	}

	for (set<MapPoint*>::iterator mpset=mspMapPoints.begin(); mpset!=mspMapPoints.end(); mpset++) {
		(*mpset)->fixConnections (this);
	}

	vector<idtype> vmvpReferenceMapPoints;
	mapArchive >> vmvpReferenceMapPoints;
	mvpReferenceMapPoints = createObjectList<MapPoint> (vmvpReferenceMapPoints);

	mapArchive >> *kfMemDb;

	mapFileFd.close();
	mbMapUpdated = true;
	cout << "Done restoring map" << endl;

	/* Point Cloud Reconstruction */
	kfCloud = pcl::PointCloud<KeyFramePt>::Ptr (new pcl::PointCloud<KeyFramePt>);
	kfCloud->width = mspKeyFrames.size();
	kfCloud->height = 1;
	kfCloud->resize(kfCloud->width);
	int p=0;
	for (set<KeyFrame*>::iterator kfi=mspKeyFrames.begin(); kfi!=mspKeyFrames.end(); kfi++) {
		KeyFrame *kf = *kfi;
		cv::Mat pos = kf->GetCameraCenter();
		kfCloud->at(p).x = pos.at<float>(0);
		kfCloud->at(p).y = pos.at<float>(1);
		kfCloud->at(p).z = pos.at<float>(2);
		kfCloud->at(p).kf = kf;
		p++;
	}
	kfOctree = pcl::octree::OctreePointCloudSearch<KeyFramePt>::Ptr (new pcl::octree::OctreePointCloudSearch<KeyFramePt> (MapOctreeResolution));
	kfOctree->setInputCloud(kfCloud);
	kfOctree->addPointsFromInputCloud();
	cout << "Done restoring Octree" << endl;
}


KeyFrame* Map::getNearestKeyFrame (const float &x, const float &y, const float &z)
{
	KeyFramePt queryPoint;
	queryPoint.x = x, queryPoint.y = y, queryPoint.z = z;

	const int k = 2;
	vector<int> idcs;
	vector<float> sqrDist;
	idcs.resize(k);
	sqrDist.resize(k);

	int r = kfOctree->nearestKSearch(queryPoint, k, idcs, sqrDist);
	if (r==0)
		return NULL;
	KeyFrame *kfn = kfCloud->at(idcs[0]).kf;
	return kfn;
}


KeyFrame* Map::offsetKeyframe(KeyFrame *kfSrc, int offset)
{
	try {
		int p = kfMapSortedId.at(kfSrc);
		p -= offset;
		return kfListSorted.at(p);
	} catch (...) {return NULL;}
}


KeyFrame* Map::offsetKeyframe (KeyFrame* kfSrc, float offset)
{
	int p = kfMapSortedId.at(kfSrc);
	int i;
	if (offset > 0)
		for (i=p; kfListSorted.at(i)->mTimeStamp < kfSrc->mTimeStamp+offset; i++);
	else
		for (i=p; kfListSorted.at(i)->mTimeStamp > kfSrc->mTimeStamp+offset; i++);
	return kfListSorted.at(i);
}

} //namespace ORB_SLAM
