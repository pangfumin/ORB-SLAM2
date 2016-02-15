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
import wx
import numpy as np
from matplotlib.figure import Figure, Axes
from matplotlib.backends.backend_wxagg import \
    FigureCanvasWxAgg as FigureCanvas
import random
from copy import copy
import sys


robotPosition = Pose()
orbListener = None
updated = False
TIMER_ID = wx.NewId ()
PF = None
orbPosition = None

# We should tune these parameters
orbError = 10
numOfParticles = 250
timeTolerance = 0.2
WheelError = 0.03
GyroError = 0.2
particleStateList = np.zeros((numOfParticles, 4))
particleAvgState = Pose()
jointPoseBroadcast = None
particleAverageList = []



    
def nrand (num) :
    r = num * np.sqrt(-2.0*np.log(random.random())) * np.cos(2.0*np.pi*random.random())
    return r
    
def odoMotionModel(particleState, move):
    global WheelError, GyroError
    if particleState.timestamp==0:
        particleState.timestamp = move['time']
        return particleState
    
    # XXX: Add randomized component to left & right wheel velocity

#    vl = move['left']*(1 + nrand(WheelError))  
    vl = move['left']*particleState.radius
    vr = move['right']*particleState.radius
#    vr = move['right']*(1 + nrand(WheelError))
    yrate = -1*(move['yawRate']+particleState.gyro_offset)*0.98 + nrand(GyroError)
    # vr = move['right] + random_vr
        
    x, y, theta = particleState.segwayMove (move['time'], vl, vr, yrate)

    newState = copy(particleState)
    newState.x = x
    newState.y = y
    newState.theta = theta
    newState.radius = particleState.radius+nrand(0.001)
    newState.gyro_offset = particleState.gyro_offset+nrand(0.0001)
    newState.timestamp = move['time']
    return newState

def odoMeasurementModel(particleOdomState, orbPose):
    global orbError
    x = particleOdomState.x
    y = particleOdomState.y
    w = np.exp(-(((x-orbPose.x)**2)/(2*orbError*orbError) + 
        ((y-orbPose.y)**2)/(2*orbError*orbError)))
    if w < 0.01:
        w = 0.01
    return w

def stateInitFunc ():
    p = Pose(0)
    p.theta = 0.5
    p.radius = 1.0+nrand(0.1)
    p.gyro_offset = 0.011+nrand(0.005)
    return p


def segwayOdomCallback (msg):
    global robotPosition, orbListener, updated, PF, numOfParticles, \
        particleStateList, orbPosition, particleAvgState, jointPoseBroadcast, particleAverageList
    
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
    
#    Try to get position from ORB
    orbTrans = None
    orbRot = None
    orbPose = None
    try:
        (orbTrans, orbRot) = orbListener.lookupTransform ('ORB_SLAM/World', 'ORB_SLAM/ExtCamera', rospy.Time(0))
        orbPosition = copy(orbTrans)
#        print (orbTrans)
        orbPose = Pose(times, orbTrans[0], orbTrans[1])
#        print ("ORB Transform found")
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
#        print ("No ORB Transform")
        pass
#    print (orbTrans)
    
    # Update particle states
    movement = {'time':times, 'left':msg.segway.left_wheel_velocity, 'right':msg.segway.right_wheel_velocity, 'yawRate':msg.segway.yaw_rate}
    PF.update (movement, orbPose)

    # XXX: In the particle state list, we ignore orientation aka. theta
    for i in range(numOfParticles):
        particleStateList[i] = [PF.particles[i].state.x, PF.particles[i].state.y, PF.particles[i].state.theta, PF.particles[i].state.radius]

    _avgState = np.insert(np.average(particleStateList, axis=0), 0, times)
    particleAverageList.append(_avgState)
    particleAvgState = Pose(times, _avgState[1], _avgState[2])
#    particleAvgState.publish(jointPoseBroadcast, 'ORB_SLAM/World', 'ORB_SLAM/Joint')
#    particleAvgState.publish
    updated = True

    
    
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
        
        if groundTruth != None:
            grnd = groundTruth.toArray(False)
            self.groundPlot, = self.ax.plot (grnd[:,0], grnd[:,1])
        self.particlePos = self.ax.scatter(particleStateList[:,0], particleStateList[:,1], s=1)
        self.robotPos = self.ax.scatter(0, 0, c='r')
        if orbPosition != None:        
            self.orbPos = self.ax.scatter(orbPosition[0], orbPosition[1], c=[[0,1,0,0.5]], s=100)
        else:
            self.orbPos = None
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
            self.particlePos.set_offsets(particleStateList[:,0:2])
            self.ax.draw_artist(self.particlePos)
            self.robotPos.set_offsets([particleAvgState.x, particleAvgState.y])
            self.ax.draw_artist(self.robotPos)
            if orbPosition != None:
                if self.orbPos == None:
                    self.orbPos = self.ax.scatter(orbPosition[0], orbPosition[1], c=[[0,1,0,0.5]], s=100)
                else:
                    self.orbPos.set_offsets([orbPosition[0], orbPosition[1]])
                self.ax.draw_artist(self.orbPos)
            self.canvas.blit(self.ax.bbox)
            
            updated = False



if __name__ == '__main__':
    
    groundTruth = PoseTable.loadCsv('/media/sujiwo/TsukubaChallenge/TsukubaChallenge/20151103/localizerResults/run2-tf-ndt.csv')
    PF = ParticleFilter (numOfParticles, stateInitFunc, odoMotionModel, odoMeasurementModel)
    jointPoseBroadcast = tf.TransformBroadcaster()

    app = wx.PySimpleApp ()
    frame = PlotFigure (groundTruth)
    tim = wx.Timer (frame, TIMER_ID)
    tim.Start(50)
    
    rospy.init_node ('SegwayORB', anonymous=True)
    orbListener = tf.TransformListener()
    rospy.Subscriber('/segway_rmp_node/segway_status', SegwayStatusStamped, segwayOdomCallback)

    frame.Show ()
    app.MainLoop ()
    
    print ("Quit, saving...")
    np.savetxt('/tmp/particleavg.csv', particleAverageList)
