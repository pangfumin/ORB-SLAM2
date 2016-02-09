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
from orbndt import Pose, PoseTable
from time import sleep
import wx
import numpy as np
from matplotlib.figure import Figure, Axes
from matplotlib.backends.backend_wxagg import \
    FigureCanvasWxAgg as FigureCanvas


robotPosition = Pose()
orbListener = None
updated = False
TIMER_ID = wx.NewId ()



def segwayOdomCallback (msg):
    global robotPosition, orbListener, updated
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
    updated = True
#    print (robotPosition.x, robotPosition.y)
    
#     Try to get position from ORB
#    try:
#        (orbTrans, orbRot) = orbListener.lookupTransform ('ORB_SLAM/World', 'ORB_SLAM/ExtCamera', rospy.Time(0))
#    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
#        print ("No ORB Transform")
#        return
#    print (orbTrans)
    
    
class PlotFigure (wx.Frame):
    
    def __init__ (self, groundTruth=None):
        wx.Frame.__init__ (self, None, wx.ID_ANY, title="Trajectory")
        
        self.fig = Figure ()
        self.canvas = FigureCanvas(self, wx.ID_ANY, self.fig)
        self.ax = self.fig.add_subplot (111)
        self.ax.set_xlim ([-100, 350])
        self.ax.set_ylim ([-100, 250])
        self.ax.set_autoscale_on (False)
        
        self.ax.grid(True)
        
        # Initial draw        
        self.data = np.zeros((1,2))
        # Please change
        self.robotPos, = self.ax.plot (self.data[:,0], self.data[:,1])
        if groundTruth != None:
            grnd = groundTruth.toArray(False)
            self.groundPlot, self.ax.plot (grnd[:,0], grnd[:,1])
        
        self.canvas.draw()
        
        # This must be done after all initial drawing
        self.bg = self.canvas.copy_from_bbox (self.ax.bbox)
        
        # Bind events to timer function
        wx.EVT_TIMER (self, TIMER_ID, self.onTimer)

        
    def onTimer (self, event):
        global updated
        """ Callback for event timer """
        if updated==True:
            self.canvas.restore_region(self.bg)
            currentRow = self.data.shape[0]
            self.data.resize ((currentRow+1, 2), refcheck=False)
            self.data[currentRow, 0] = robotPosition.x
            self.data[currentRow, 1] = robotPosition.y
            self.robotPos.set_ydata(self.data[:,1])
            self.robotPos.set_xdata(self.data[:,0])
            self.ax.draw_artist(self.robotPos)
            self.canvas.blit(self.ax.bbox)
            updated = False
            



if __name__ == '__main__':
    
    groundTruth = PoseTable.loadCsv('/media/sujiwo/TsukubaChallenge/TsukubaChallenge/20151103/localizerResults/run2-tf-ndt.csv')

    app = wx.PySimpleApp (groundTruth)
    frame = PlotFigure ()
    tim = wx.Timer (frame, TIMER_ID)
    tim.Start(50)
    
    rospy.init_node ('SegwayORB', anonymous=True)
    orbListener = tf.TransformListener()
    rospy.Subscriber('/segway_rmp_node/segway_status', SegwayStatusStamped, segwayOdomCallback)

    frame.Show ()
    app.MainLoop ()
