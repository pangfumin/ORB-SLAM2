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
orbError = 0.5
orbYawError = 15* np.pi/180
numOfParticles = 250
timeTolerance = 0.2
WheelError = 0.03
GyroError = 0.5
particleStateList = np.zeros((numOfParticles, 5))
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
    vl = move['left']*particleState.radius+ nrand(WheelError)
    vr = move['right']*particleState.radius+ nrand(WheelError)
#    vr = move['right']*(1 + nrand(WheelError))
    yrate = -1*(move['yawRate']+particleState.gyro_offset) + nrand(GyroError)
    # vr = move['right] + random_vr
        
    x, y, theta = particleState.segwayMove (move['time'], vl, vr, yrate)

    newState = copy(particleState)
    newState.x = x
    newState.y = y
    newState.theta = theta
    newState.radius = particleState.radius+nrand(0.0001)
    newState.gyro_offset = particleState.gyro_offset#+nrand(0.00001)
    newState.timestamp = move['time']
    return newState

def odoMeasurementModel(particleOdomState, orbPose):
    global orbError, orbYawError
    
    x = particleOdomState.x
    y = particleOdomState.y
    t = particleOdomState.theta
    orbYaw = orbPose.euler()[2]
    diff = t-orbYaw
    if diff>np.pi:
        diff = diff - 2*np.pi
    if diff<-np.pi:
        diff = diff + 2*np.pi
    w = np.exp(-(((x-orbPose.x)**2)/(2*orbError*orbError) + 
        ((y-orbPose.y)**2)/(2*orbError*orbError) +
        (diff**2)/(2*orbYawError*orbYawError)
        ))
    if w < 1e-8:
        w = 1e-8
    return w
    
    
def odoMeasurementModel2 (particleOdomState, *orbPoses):
    global orbError
    x = particleOdomState.x
    y = particleOdomState.y
    ws = []
    for pose in orbPoses:
        wx = np.exp(-(((x-pose.x)**2)/(2*orbError*orbError) + 
            ((y-pose.y)**2)/(2*orbError*orbError)))
        ws.append(wx)
    # XXX: Tweak this line to incorporate other type of probability selection
    w = np.max(ws)
    return w


def stateInitFunc ():
    p = Pose(0, 3.6+nrand(0.5), 1.5+nrand(0.5))
    p.theta = 0.5
    p.radius = 1.0+nrand(0.05)
    p.gyro_offset = 0.0114#+nrand(0.005)
    return p


msgBucket = []
msgBucketMax = 10

orbLastTimeFix = -1
orbTimeTolerance = 0.25


def segwayOdomCallback (msg):
    global robotPosition, orbListener, updated, PF, numOfParticles, \
        particleStateList, orbPosition, particleAvgState, jointPoseBroadcast, particleAverageList, \
        msgBucket, msgBucketMax, \
        orbLastTimeFix, orbTimeTolerance
    
    times = msg.header.stamp.to_sec()
    
    if robotPosition.timestamp==0:
        robotPosition.timestamp = times
        robotPosition.theta = 0.0
        return
        
#    if len(msgBucket < msgBucketMax):
#        msgBucket.append([msg.segway.left_wheel_velocity, msg.segway.right_wheel_velocity])
        
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
        
        # XXX: Need to verify ORB pose value as valid (no later than orbTimeTolerance)
        
        (orbTrans, orbRot) = orbListener.lookupTransform ('ORB_SLAM/World', 'ORB_SLAM/Camera1', rospy.Time(0))
        orbPosition = copy(orbTrans)
        orbPose = Pose(times, orbTrans[0], orbTrans[1], orbTrans[2], orbRot[0], orbRot[1], orbRot[2], orbRot[3])
        if orbLastTimeFix < 0:
            orbLastTimeFix = rospy.Time.now().to_sec()
        else :
            ct = rospy.Time.now().to_sec()
            td = ct - orbLastTimeFix
#            mprint ("TDif:{}, tolerance:{}".format(td, orbTimeTolerance))
            if (abs(td) > orbTimeTolerance) :
                # discard ORB
                orbTrans = None
                orbRot = None
                orbPose = None
                raise Exception
            else:
                orbLastTimeFix = ct
            
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException, Exception):
        print ("ORB Lost Tracking")
        pass
    
    # Update particle states
    movement = {'time':times, 'left':msg.segway.left_wheel_velocity, 'right':msg.segway.right_wheel_velocity, 'yawRate':msg.segway.yaw_rate}
    PF.update (movement, orbPose)

    # XXX: In the particle state list, we ignore orientation aka. theta
    for i in range(numOfParticles):
        particleStateList[i] = [
            PF.particles[i].state.x, 
            PF.particles[i].state.y, 
            PF.particles[i].state.theta, 
            PF.particles[i].state.radius, 
            PF.particles[i].state.gyro_offset]

    _avgState = np.insert(np.average(particleStateList, axis=0), 0, times)
    particleAverageList.append(_avgState)
    # 0 : timestamp
    # 1 : X
    # 2 : Y
    # 3 : Yaw (theta)
    # 4 : wheel radius
    # 5 : Gyro offset
    
    # Yaw was previously in radian
#    yaw = _avgState[3]
#    yaw = yaw*180/np.pi
#    if (yaw > 0):
#        yaw = yaw % 360.0
#    else :
#        yaw = - (abs(yaw) % 360)
#    if orbPose != None:
#        eangle = orbPose.euler() * 180.0/np.pi
#    else:
#        eangle = [0, 0, 0]
#    print ("Odo Yaw = {} degrees, ORB Yaw = {} degrees".format(yaw, str(eangle)))
    
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
    rospy.Subscriber('/segway_rmp_node/segway_status', SegwayStatusStamped, segwayOdomCallback, queue_size=1)

    frame.Show ()
    app.MainLoop ()
    
    print ("Quit, saving...")
    np.savetxt('/tmp/particleavg.csv', particleAverageList)
