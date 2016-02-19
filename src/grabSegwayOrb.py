#!/usr/bin/python

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
import multiprocessing.dummy as mp


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

# Runtime values
particleStateList = np.zeros((numOfParticles, 5))
particleAvgState = Pose()
jointPoseBroadcast = None
particleAverageList = []
orbProcess1 = None
orbProcess2 = None

# For Run 1
odomInitState = {'x':3.6, 'y':1.5, 'theta': 0.5}

# For Run 2
#odomInitState = {'x':0, 'y':-0.5, 'theta':0.5}

    
def nrand (num) :
    r = num * np.sqrt(-2.0*np.log(random.random())) * np.cos(2.0*np.pi*random.random())
    return r
    
def odoMotionModel(particleState, move):
    global WheelError, GyroError
    if particleState.timestamp==0:
        particleState.timestamp = move['time']
        return particleState
    
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
    angleDiff = t-orbYaw
    if angleDiff > np.pi:
        angleDiff = angleDiff - 2*np.pi
    if angleDiff < -np.pi:
        angleDiff = angleDiff + 2*np.pi
    w = np.exp(-(((x-orbPose.x)**2)/(2*orbError*orbError) + 
        ((y-orbPose.y)**2)/(2*orbError*orbError) +
        (angleDiff**2)/(2*orbYawError*orbYawError)
        ))
    if w < 1e-8:
        w = 1e-8
    return w
    
    
def odoMeasurementModel2 (particleOdomState, *orbPoses):
    global orbError, orbYawError
    
    x = particleOdomState.x
    y = particleOdomState.y
    t = particleOdomState.theta
    ws = []

    for pose in orbPoses:
        if pose==None:
            continue

        orbYaw = pose.euler()[2]
        angleDiff = t-orbYaw
        if angleDiff > np.pi:
            angleDiff = angleDiff - 2*np.pi
        if angleDiff < -np.pi:
            angleDiff = angleDiff + 2*np.pi

        wx = np.exp(-(((x-pose.x)**2)/(2*orbError*orbError) + 
            ((y-pose.y)**2)/(2*orbError*orbError) +
            (angleDiff**2)/(2*orbYawError*orbYawError)
            ))
        ws.append(wx)

    # XXX: Tweak this line to incorporate other type of probability selection
    w = np.max(ws)
    return max(w, 1e-8)


def stateInitFunc ():
    global odomInitState
    initialPositionNoise = 0.1
    
    p = Pose(0, odomInitState['x']+nrand(initialPositionNoise), odomInitState['y']+nrand(initialPositionNoise))
    p.theta = odomInitState['theta']
    p.radius = 1.0+nrand(0.05)
    p.gyro_offset = 0.0114#+nrand(0.005)
    return p


msgBucket = []
msgBucketMax = 10

orbLastTimeFix = -1
orbTimeTolerance = 0.25


def segwayOdomCallback (msg):
    global robotPosition, orbListener, updated, PF, numOfParticles, \
        particleStateList, particleAvgState, jointPoseBroadcast, particleAverageList, \
        msgBucket, msgBucketMax, \
        orbLastTimeFix, orbTimeTolerance, \
        orbProcess1, orbProcess2
    
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
    orbPose1 = orbProcess1.getPose()
    orbPose2 = orbProcess2.getPose()
    
    # Update particle states
    movement = {'time':times, 'left':msg.segway.left_wheel_velocity, 'right':msg.segway.right_wheel_velocity, 'yawRate':msg.segway.yaw_rate}
    if (orbPose1==None and orbPose2==None):
        PF.update (movement)
    else:
        PF.update (movement, orbPose1, orbPose2)

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
    
    
class OrbCollector:
    def __init__ (self, parentFrameId, childFrameId, tfListener):
        self.listener = tfListener
        self.parentFrameId = parentFrameId
        self.childFrameId = childFrameId
        # Time delay before timeout
        self.timeTolerance = 0.1
        self.translation = np.zeros(3)
        self.rotation = np.zeros(4)
        self.time = -1
        self.stop = False
        self.process = mp.Process(target=self.start)
        self.process.start()

        
    def getPose (self):
        if self.time == -1:
            return None
        return Pose(self.time, \
            self.translation[0], self.translation[1], self.translation[2], \
            self.rotation[0], self.rotation[1], self.rotation[2], self.rotation[3])


    def start (self):
        print ("{} started".format(self.childFrameId))
        while (self.stop != True):
            try:
                tm = rospy.Time.now()
#                print ("Trying lookup")
                self.listener.waitForTransform (self.parentFrameId, self.childFrameId, tm, rospy.Duration(self.timeTolerance))
                (trans, rot) = self.listener.lookupTransform (self.parentFrameId, self.childFrameId, tm)
#                print ("Got ORB for {}".format(self.childFrameId))
                self.time = tm.to_sec()
                self.translation[0] = trans[0]
                self.translation[1] = trans[1]
                self.translation[2] = trans[2]
                self.rotation[0] = rot[0]
                self.rotation[1] = rot[1]
                self.rotation[2] = rot[2]
                self.rotation[3] = rot[3]
            except Exception:
                self.time = -1


class PlotFigure (wx.Frame):
    
    def __init__ (self, groundTruth=None):
        wx.Frame.__init__ (self, None, wx.ID_ANY, title="Trajectory")
        
        self.fig = Figure ()
        self.canvas = FigureCanvas(self, wx.ID_ANY, self.fig)
        self.ax = self.fig.add_subplot (111)
        self.ax.set_xlim ([-100, 350])
        self.ax.set_ylim ([-100, 250])
        self.ax.set_autoscale_on (False)
        self.orbPos1 = None
        self.orbPos2 = None
        self.ax.grid(True)
        
        if groundTruth != None:
            grnd = groundTruth.toArray(False)
            self.groundPlot, = self.ax.plot (grnd[:,0], grnd[:,1])
        self.particlePos = self.ax.scatter(particleStateList[:,0], particleStateList[:,1], s=1)
        self.robotPos = self.ax.scatter(odomInitState['x'], odomInitState['y'], c='r', linewidths=0)
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
            
            orbPosition1 = orbProcess1.getPose()
            orbPosition2 = orbProcess2.getPose()
            if orbPosition1 != None:
                if self.orbPos1 == None:
                    self.orbPos1 = self.ax.scatter(orbPosition1.x, orbPosition1.y, c=[[0,1,0,0.5]], s=100, linewidths=0)
                else:
                    self.orbPos1.set_offsets([orbPosition1.x, orbPosition1.y])
                self.ax.draw_artist(self.orbPos1)
            if orbPosition2 != None:
                if self.orbPos2 == None:
                    self.orbPos2 = self.ax.scatter(orbPosition2.x, orbPosition2.y, c=[[0,0,1,0.5]], s=100, linewidths=0)
                else:
                    self.orbPos2.set_offsets([orbPosition2.x, orbPosition2.y])
                self.ax.draw_artist(self.orbPos2)


            self.canvas.blit(self.ax.bbox)
            updated = False



#if __name__ == '__main__' :
#    import time
#    
#    rospy.init_node('SegwayORB', anonymous=True)
#    tfListener = tf.TransformListener()
#    
#    orbProcess1 = OrbCollector('ORB_SLAM/World', 'ORB_SLAM/Camera1', tfListener)
#    orbProcess2 = OrbCollector('ORB_SLAM/World', 'ORB_SLAM/Camera2', tfListener)
#    
#    while (True):
#        cam1 = orbProcess1.getPose()
#        cam2 = orbProcess2.getPose()
#        if (cam1 != None):
#            print ("Camera 1: {}".format(str(cam1)))
#        if (cam2 != None):
#            print ("Camera 2: {}".format(str(cam2)))
#        time.sleep (0.1)



if __name__ == '__main__':
    
    from sys import argv
    
#    groundTruth = PoseTable.loadCsv('/media/sujiwo/TsukubaChallenge/TsukubaChallenge/20151103/localizerResults/run2-tf-ndt.csv')
    if (len(argv)<2):
        print ('Enter ground truth table')
        sys.exit(-1)

    groundTruth = PoseTable.loadCsv(argv[1])
    
    PF = ParticleFilter (numOfParticles, stateInitFunc, odoMotionModel, odoMeasurementModel2)
    jointPoseBroadcast = tf.TransformBroadcaster()

    app = wx.PySimpleApp ()
    frame = PlotFigure (groundTruth)
    tim = wx.Timer (frame, TIMER_ID)
    tim.Start(50)
    
    rospy.init_node ('SegwayORB', anonymous=True)
    orbListener = tf.TransformListener()
    
    orbProcess1 = OrbCollector ('ORB_SLAM/World', 'ORB_SLAM/Camera1', orbListener)
    orbProcess2 = OrbCollector ('ORB_SLAM/World', 'ORB_SLAM/Camera2', orbListener)
    
    rospy.Subscriber('/segway_rmp_node/segway_status', SegwayStatusStamped, segwayOdomCallback, queue_size=1)

    frame.Show ()
    app.MainLoop ()
    
    orbProcess1.stop = True
    orbProcess2.stop = True
    print ("Quit, saving...")
    np.savetxt('/tmp/particleavg.csv', particleAverageList)
