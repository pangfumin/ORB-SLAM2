#!/usr/bin/python

import os
import sys
import rospy
import rosbag
import cv2
import cv
import numpy as np
from sensor_msgs.msg import Image as rImage
from sensor_msgs.msg import CompressedImage as cImage


encoding = 'MJPG'
fps = 20.0
outputName = 'output.avi'
isColor = True

if __name__ == '__main__' :
    
    topicName = sys.argv[2]
    if (topicName[-10:] == 'compressed') :
        raw = False
    else :
        raw = True

    videoWriter = None
    mybag = rosbag.Bag (sys.argv[1], 'r')
    countMessage = 0
    topicInfo = mybag.get_type_and_topic_info(topicName)
    numMessage = topicInfo[1][topicName].message_count

    for topic, msg, tm in mybag.read_messages(topics=[topicName]) :
        
        if (raw==False) :
            imgarray = np.fromstring(msg.data, dtype=np.uint8) 
            imageData = cv2.imdecode(imgarray, -1)
            if (len(imageData.shape)==2) :
                isColor = False
            else :
                isColor = True
        else :
            pass
        if (videoWriter is None) :
            videoWriter = cv2.VideoWriter(outputName, cv.CV_FOURCC(
                encoding[0], 
                encoding[1], 
                encoding[2], 
                encoding[3]),
                fps, (imageData.shape[1], imageData.shape[0]), isColor)
            print ("Size: {}x{}".format(imageData.shape[1], imageData.shape[0]))
            m = videoWriter.isOpened() 
        videoWriter.write (imageData)
        countMessage += 1
        if (countMessage == 2000) :
            break
#         print ("{}/{}\r".format(countMessage, numMessage)),
        print (tm)
    
    videoWriter.release()
    print (".. Done")
    
    
    pass