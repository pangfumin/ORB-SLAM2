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

#ifndef MAPPOINT_H
#define MAPPOINT_H

#include <string>
#include <map>
#include <vector>
#include <set>
#include <opencv2/core/core.hpp>
#include <boost/thread.hpp>
#include <boost/serialization/serialization.hpp>
#include <boost/assert.hpp>


typedef int64_t idtype;


namespace boost {
namespace serialization {

template <class Archive>
	void save (Archive &, const ORB_SLAM::MapPoint &, const unsigned int);
template <class Archive>
	void load (Archive &, ORB_SLAM::MapPoint &, const unsigned int);

}
}


namespace ORB_SLAM
{

class ImageFeature;
class KeyFrame;
class Map;

using std::string;
using std::set;
using std::vector;
using std::map;

class MapPoint
{
public:
    MapPoint(const cv::Mat &Pos, KeyFrame* pRefKF, Map* pMap);

    MapPoint ()
    {
    	BOOST_ASSERT ((mbTrackInView=false) || true);
    	BOOST_ASSERT ((mbBad=false) || true);
    }

    void SetWorldPos(const cv::Mat &Pos);
    cv::Mat GetWorldPos();

    cv::Mat GetNormal();
    KeyFrame* GetReferenceKeyFrame();

    std::map<KeyFrame*,size_t> GetObservations();
    int Observations();

    void AddObservation(KeyFrame* pKF,size_t idx);
    void EraseObservation(KeyFrame* pKF);

    int GetIndexInKeyFrame(KeyFrame* pKF);
    bool IsInKeyFrame(KeyFrame* pKF);

    void SetBadFlag();
    bool isBad();

    void Replace(MapPoint* pMP);

    void IncreaseVisible();
    void IncreaseFound();
    float GetFoundRatio();

    void ComputeDistinctiveDescriptors();

    cv::Mat GetDescriptor();

    void UpdateNormalAndDepth();

    float GetMinDistanceInvariance();
    float GetMaxDistanceInvariance();

public:
    long unsigned int mnId;
    static long unsigned int nNextId;
    long int mnFirstKFid;

    // Variables used by the tracking
    float mTrackProjX;
    float mTrackProjY;
    bool mbTrackInView;
    int mnTrackScaleLevel;
    float mTrackViewCos;
    long unsigned int mnTrackReferenceForFrame;
    long unsigned int mnLastFrameSeen;

    // Variables used by local mapping
    long unsigned int mnBALocalForKF;
    long unsigned int mnFuseCandidateForKF;

    // Variables used by loop closing
    long unsigned int mnLoopPointForKF;
    long unsigned int mnCorrectedByKF;
    long unsigned int mnCorrectedReference;

protected:

    template <class Archive>
    friend void boost::serialization::save (Archive &, const ORB_SLAM::MapPoint &, const unsigned int);

    template <class Archive>
    friend void boost::serialization::load (Archive &, ORB_SLAM::MapPoint &, const unsigned int);

     // Position in absolute coordinates
     cv::Mat mWorldPos;

     // Keyframes observing the point and associated index in keyframe
     std::map<KeyFrame*,size_t> mObservations;

     // Mean viewing direction
     cv::Mat mNormalVector;

     // Best descriptor to fast matching
     cv::Mat mDescriptor;

     // Reference KeyFrame
     KeyFrame* mpRefKF;

     // Tracking counters
     int mnVisible;
     int mnFound;

     // Bad flag (we do not currently erase MapPoint from memory)
     bool mbBad;

     // Scale invariance distances
     float mfMinDistance;
     float mfMaxDistance;

     Map* mpMap;

     boost::mutex mMutexPos;
     boost::mutex mMutexFeatures;


public:
     // Additions for map restoration
	static map<idtype, MapPoint*> objectListLookup;
	void fixConnections (Map *smap);
};

} //namespace ORB_SLAM




#endif // MAPPOINT_H
