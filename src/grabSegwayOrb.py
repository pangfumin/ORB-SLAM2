# -*- coding: utf-8 -*-
"""
Created on Mon Feb  8 13:14:56 2016

@author: sujiwo
"""

from __future__ import division
import rospy
from filter import ParticleFilter
from segway_rmp.msg import SegwayStatusStamped
import tf
from orbndt import Pose
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation


robotPosition = Pose()
orbListener = None


def segwayOdomCallback (msg):
    global robotPosition, orbListener
    times = msg.header.stamp.to_sec()
    
    if robotPosition.timestamp==0:
        robotPosition.timestamp = times
        robotPosition.theta = 0.0
        return
    robotPosition.x, \
    robotPosition.y, \
    robotPosition.theta = robotPosition.segwayMove(times, 
        msg.segway.left_wheel_velocity, 
        msg.segway.right_wheel_velocity, 
        msg.segway.yaw_rate)
    robotPosition.timestamp = times
    print (robotPosition.x, robotPosition.y)
    
#     Try to get position from ORB
#    try:
#        (orbTrans, orbRot) = orbListener.lookupTransform ('ORB_SLAM/World', 'ORB_SLAM/ExtCamera', rospy.Time(0))
#    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
#        print ("No ORB Transform")
#        return
#    print (orbTrans)
    
    
class TrajectoryPlot (object):
    def __init__ (self, pose):
        self.figure, self.axes = plt.subplots()
        self.axes.set_xlim(-100, 350)
        self.axes.set_ylim(-100, 250)
        self.axes.grid(True)
        self.odomPlot, = self.axes.plot([], [])
        self.data = []
        self.pose = pose

    def init (self):
        self.odomPlot.set_data([], [])
        return self.odomPlot,
    
    def __call__ (self, i):
        if i==0:
            return self.init()
        if robotPosition.timestamp==0:
            return self.odomPlot
        self.data.append([robotPosition.x, robotPosition.y])
        self.odomPlot.set_data(self.data)
        return self.odomPlot,


if __name__ == '__main__':
    
    fig = plt.figure()
    ax = plt.axes(xlim=(-100,350), ylim=(-100,250))
    line, = ax.plot([], [])
    
    def plotInit ():
        line.set_data([], [])
        return line,
        
    def plotAnim (i):
        return line,
    
    anim = FuncAnimation (fig, plotAnim, init_func=plotInit, interval=10)
    plt.draw()
    plt.show()
    
    rospy.init_node ('SegwayORB', anonymous=True)
    orbListener = tf.TransformListener()
    rospy.Subscriber('/segway_rmp_node/segway_status', SegwayStatusStamped, segwayOdomCallback)
    rospy.spin()