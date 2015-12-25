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

#ifndef MAP_H
#define MAP_H

#include <set>
#include <string>
#include <opencv2/opencv.hpp>
#include <boost/thread.hpp>
#include <sqlite3.h>
#include <exception>

// For Fast keyframe search
#define PCL_NO_PRECOMPILE
#include <pcl/point_cloud.h>
#include <pcl/octree/octree.h>
#include <pcl/octree/impl/octree_search.hpp>


namespace ORB_SLAM
{

class MapPoint;
class KeyFrame;
class KeyFrameDatabase;


struct KeyFramePt {
	PCL_ADD_POINT4D;
	ORB_SLAM::KeyFrame *kf;
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16 ;


class Map
{
public:
    Map();

    void AddKeyFrame(KeyFrame* pKF);
    void AddMapPoint(MapPoint* pMP);
    void EraseMapPoint(MapPoint* pMP);
    void EraseKeyFrame(KeyFrame* pKF);
    void SetCurrentCameraPose(cv::Mat Tcw);
    void SetReferenceKeyFrames(const std::vector<KeyFrame*> &vpKFs);
    void SetReferenceMapPoints(const std::vector<MapPoint*> &vpMPs);

    std::vector<KeyFrame*> GetAllKeyFrames();
    std::vector<MapPoint*> GetAllMapPoints();
    cv::Mat GetCameraPose();
    std::vector<KeyFrame*> GetReferenceKeyFrames();
    std::vector<MapPoint*> GetReferenceMapPoints();

    int MapPointsInMap();
    int KeyFramesInMap();

    void SetFlagAfterBA();
    bool isMapUpdated();
    void ResetUpdated();

    unsigned int GetMaxKFid();

    void clear();

    class BadMapFile : public std::exception
	{};
    class MapFileException : public std::exception
	{};

    void saveToDisk (const std::string &filename, KeyFrameDatabase *kfMemDb);
    void loadFromDisk (const std::string &filename, KeyFrameDatabase *kfMemDb=NULL);

	struct MapFileHeader {
		char signature[7];
		long unsigned int
			numOfKeyFrame,
			numOfMapPoint,
			numOfReferencePoint;
	};

//	volatile static
	bool MapIsReadonly;

	KeyFrame* getNearestKeyFrame (const float &x, const float &y, const float &z);
	KeyFrame* offsetKeyframe (KeyFrame* kfSrc, int offset);
	std::vector<KeyFrame*> kfListSorted;
	std::map<KeyFrame*, int> kfMapSortedId;

protected:
    std::set<MapPoint*> mspMapPoints;
    std::set<KeyFrame*> mspKeyFrames;

    std::vector<MapPoint*> mvpReferenceMapPoints;

    unsigned int mnMaxKFid;

    boost::mutex mMutexMap;
    bool mbMapUpdated;
    void createMapFileHeader (MapFileHeader &hdr);

    pcl::PointCloud<KeyFramePt>::Ptr kfCloud;
    pcl::octree::OctreePointCloudSearch<KeyFramePt>::Ptr kfOctree;
};

} //namespace ORB_SLAM

#endif // MAP_H
