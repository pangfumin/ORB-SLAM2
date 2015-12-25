import os
import sys
import rospy
import cv2
import cv
import numpy as np
from sensor_msgs.msg import Image as rImage
from sensor_msgs.msg import CompressedImage as cImage
from std_msgs.msg import Header as rHeader


videoFilename = "/home/sujiwo/Data/NewCollege/newCollegeLeftCamera.avi"
topic = "/newcollege/stereo/left/image"
seek = 30.0

if __name__ == '__main__' :
    
    video = cv2.VideoCapture(videoFilename)
    if (not video.isOpened()) :
        raise IOError("Unable to open video")
    fps = video.get (cv.CV_CAP_PROP_FPS)
    video.set (cv2.cv.CV_CAP_PROP_POS_MSEC, seek*1000.0)
    
    imgShow = rospy.Publisher (topic, cImage, queue_size=10)
    rospy.init_node ('videoShow', anonymous=True)

    rate = rospy.Rate (fps)
    
    while not (rospy.is_shutdown()) :
        ret, frame = video.read ()
        ret, cframe = cv2.imencode ('.png', frame)
        
        header = rHeader (stamp=rospy.Time.now(), frame_id='stereo_camera')
        imagemsg = cImage (header=header, format='png', data=str(cframe.data))
        imgShow.publish(imagemsg)
        rate.sleep()
    
    video.release ()