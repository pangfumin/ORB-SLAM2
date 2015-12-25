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

#include <cstdlib>
#include <iostream>
#include <fstream>
#include <ros/ros.h>
#include <ros/package.h>
#include <boost/thread.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <signal.h>
#include <opencv2/core/core.hpp>
#include <boost/program_options.hpp>

#include "Tracking.h"
#include "FramePublisher.h"
#include "Map.h"
#include "MapPublisher.h"
#include "LocalMapping.h"
#include "LoopClosing.h"
#include "KeyFrameDatabase.h"
#include "ORBVocabulary.h"


#include "Converter.h"
#include <sys/time.h>
#include <sys/select.h>
#include <unistd.h>



using namespace std;
using namespace boost::posix_time;
namespace po = boost::program_options;


volatile bool stopChild = false;
bool readMap;
volatile bool ctrlBreakPressed = false;
string inputVideoFile ("/home/sujiwo/Data/NewCollege/NewCollegeStereoLeft.avi");
const string mapPathStd ("Data/Map.db");
string mapPath;


void ctrlBreakFunc (int signalNum)
{
	ctrlBreakPressed = true;
}


int main(int argc, char **argv)
{
    cout << endl << "ORB-SLAM Copyright (C) 2014 Raul Mur-Artal" << endl <<
            "This program comes with ABSOLUTELY NO WARRANTY;" << endl  <<
            "This is free software, and you are welcome to redistribute it" << endl <<
            "under certain conditions. See LICENSE.txt." << endl;

    po::options_description orbSlamOptions ("Options");
    orbSlamOptions.add_options()
		("help", "produce help message")
		("vocabulary", po::value<string>(), "ORB Vocabulary file")
		("config", po::value<string>(), "Configuration file")
		("video", po::value<string>(), "Load images from this video file instead of ROS")
		("map", po::value<string>(), "Use this map file")
		("seek", po::value<float>(), "If loading video, seek to this second")
		("ro", "Do not modify map")
    ;
    po::variables_map orbSlamOptMap;
    po::store (po::parse_command_line(argc, argv, orbSlamOptions), orbSlamOptMap);
    po::notify (orbSlamOptMap);

    if (orbSlamOptMap.count("help") or !orbSlamOptMap.count("config")) {
//        cerr << endl << "Usage: rosrun ORB_SLAM ORB_SLAM path_to_vocabulary path_to_settings (absolute or relative to package directory)" << endl;
        cout << orbSlamOptions << endl;
        return 1;
    }

    ros::init(argc, argv, "ORB_SLAM");
    ros::start();

    // Load Settings and Check
    string strSettingsFile = orbSlamOptMap["config"].as<string>();
    cv::FileStorage fsSettings(strSettingsFile.c_str(), cv::FileStorage::READ);
    if(!fsSettings.isOpened())
    {
        ROS_ERROR("Wrong path to settings. Path must be absolute or relative to ORB_SLAM package directory.");
        ros::shutdown();
        return 1;
    }

    // New version to load vocabulary from text file "Data/ORBvoc.txt". 
    // If you have an own .yml vocabulary, use the function
    // saveToTextFile in Thirdparty/DBoW2/DBoW2/TemplatedVocabulary.h
    string strVocFile = orbSlamOptMap["vocabulary"].as<string>();
    cout << endl << "Loading ORB Vocabulary. This could take a while." << endl;
    
    ORB_SLAM::ORBVocabulary Vocabulary;
    bool bVocLoad = Vocabulary.loadFromTextFile(strVocFile);
    if(!bVocLoad)
    {
        cerr << "Wrong path to vocabulary. Path must be absolute or relative to ORB_SLAM package directory." << endl;
        cerr << "Failed to open at: " << strVocFile << endl;
        ros::shutdown();
        return 1;
    }

    cout << "Vocabulary loaded!" << endl << endl;

    //Create KeyFrame Database
    ORB_SLAM::KeyFrameDatabase Database(Vocabulary);

    //Create the map
    ORB_SLAM::Map World;
	if (orbSlamOptMap.count("ro"))
		World.MapIsReadonly = true;
	if (orbSlamOptMap.count("map")) {
		mapPath = orbSlamOptMap["map"].as<string>();
	}
	else {
		mapPath = mapPathStd;
	}
	// try loading map
	try {
		cout << "Loading map " << mapPath << endl;
		World.loadFromDisk (mapPath, &Database);
		readMap = true;
		cout << "Done loading" << endl;
	} catch (exception &e) {}

    //Create Frame Publisher for image_view
    ORB_SLAM::FramePublisher FramePub;
    FramePub.SetMap(&World);

    //Create Map Publisher for Rviz
    ORB_SLAM::MapPublisher MapPub(&World);
    if (readMap==true) {
		MapPub.keepPublish = true;
		MapPub.Refresh();
    }

    //Initialize the Tracking Thread and launch
    ORB_SLAM::Tracking Tracker(&Vocabulary, &FramePub, &MapPub, &World, strSettingsFile);
    boost::thread trackingThread;
    if (readMap==true)
    	Tracker.setMapLoaded();
    if (orbSlamOptMap.count("video")) {
    	string videoFile = orbSlamOptMap["video"].as<string> () ;
    	float seek;
    	if (orbSlamOptMap.count("seek"))
    		seek = orbSlamOptMap["seek"].as<float> ();
    	else seek = 0;
    	trackingThread = boost::thread(&ORB_SLAM::Tracking::RunFromVideoFile, &Tracker, videoFile, seek);
    }
    else {
    	trackingThread = boost::thread(&ORB_SLAM::Tracking::RunFromROS, &Tracker);
    }

    Tracker.SetKeyFrameDatabase(&Database);

    //Initialize the Local Mapping Thread and launch
    ORB_SLAM::LocalMapping LocalMapper(&World);
    boost::thread localMappingThread(&ORB_SLAM::LocalMapping::Run,&LocalMapper);

    //Initialize the Loop Closing Thread and launch
    ORB_SLAM::LoopClosing LoopCloser(&World, &Database, &Vocabulary);
    boost::thread loopClosingThread(&ORB_SLAM::LoopClosing::Run, &LoopCloser);

    //Set pointers between threads
    Tracker.SetLocalMapper(&LocalMapper);
    Tracker.SetLoopClosing(&LoopCloser);

    LocalMapper.SetTracker(&Tracker);
    LocalMapper.SetLoopCloser(&LoopCloser);

    LoopCloser.SetTracker(&Tracker);
    LoopCloser.SetLocalMapper(&LocalMapper);

    //This "main" thread will show the current processed frame and publish the map
    float fps = fsSettings["Camera.fps"];
    if(fps==0)
        fps=30;

    ros::Rate r(fps);
    cout << "Start mapping; press <Ctrl+C> to stop" << endl;
    signal (SIGINT, ctrlBreakFunc);

    // For debugging: limit running duration to a value (in seconds) specified here
//    time_duration maxRunningTime (0, 0, 40);
//    ptime T1 = second_clock::local_time();

    while (true)
    {
        FramePub.Refresh();
        MapPub.Refresh();
        Tracker.CheckResetByPublishers();

        if (ctrlBreakPressed==true)
        	break;

//        ptime T2 = second_clock::local_time();
//        if (T1 + maxRunningTime < T2)
//        	break;

        r.sleep();
    }

    cout << "Done mapping" << endl;
    stopChild = true;
    trackingThread.join();
    localMappingThread.join();
    loopClosingThread.join();

    if (World.MapIsReadonly==false) {
    	cout << "Saving map..." << endl;
    	World.saveToDisk(mapPath.c_str(), &Database);
    }

    ros::shutdown();

	return 0;
}
